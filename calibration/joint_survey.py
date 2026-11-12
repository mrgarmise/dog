#!/usr/bin/env python3
"""Freenove FNK0050 joint characterization log (NO HARDWARE CONTROL).

This program cannot determine actual angles or speed from the stock servos:
there is no position feedback. Records observations from a human/operator
after separate qualified low-risk hardware testing, and exports joint limits.
Never imports Servo, PCA9685, smbus, RPi.GPIO or touches motor hardware.
"""
import argparse
import copy
import json
import math
import os
import sys
from datetime import datetime, timezone
from pathlib import Path

CHANNELS = ((4, 3, 2), (7, 6, 5), (8, 9, 10), (11, 12, 13))
ALL_CHANNELS = [x for leg in CHANNELS for x in leg]
DRIVER_BOUNDS = (18.0, 162.0)
DEFAULT_FILE = Path(__file__).resolve().parent / "joint_measurements.json"

def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="seconds")

def valid_number(value, label):
    try:
        x = float(value)
    except (ValueError, TypeError):
        raise ValueError(f"{label} must be numeric") from None
    if not math.isfinite(x):
        raise ValueError(f"{label} must be finite")
    return x

def check_channel(ch):
    if ch not in ALL_CHANNELS:
        raise ValueError(f"Channel {ch} is not a known leg servo (2..13 in specific order)")

def blank_record():
    joints = {}
    for leg, group in enumerate(CHANNELS):
        for joint, ch in enumerate(group):
            joints[str(ch)] = {
                "leg_index": leg,
                "joint_index": joint,
                "physical_corner": None,
                "neutral_command_deg": None,
                "lower_observed_deg": None,
                "upper_observed_deg": None,
                "margin_deg": None,
                "speed_observations": [],
                "notes": "",
            }
    return {
        "schema_version": 1,
        "robot": "Freenove FNK0050",
        "mode": "measurement_log_only_NO_HARDWARE",
        "angle_convention": "Freenove Servo.py commanded angle (not measured shaft angle)",
        "disclaimer": "Limits are human-observed estimates; not verified hard stops or guaranteed safe limits.",
        "created_utc": utc_now(),
        "joints": joints,
    }

def validate(doc):
    if doc.get("schema_version") != 1 or doc.get("mode") != "measurement_log_only_NO_HARDWARE":
        raise ValueError("Unrecognized joint measurement log")
    if set(doc.get("joints", {})) != {str(ch) for ch in ALL_CHANNELS}:
        raise ValueError("Incorrect set of channels")
    for s, j in doc["joints"].items():
        lo, hi = j.get("lower_observed_deg"), j.get("upper_observed_deg")
        for v in (lo, hi):
            if v is not None and not (DRIVER_BOUNDS[0] <= valid_number(v,"limit") <= DRIVER_BOUNDS[1]):
                raise ValueError(f"Channel {s}: observation outside original driver range")
        if lo is not None and hi is not None and lo >= hi:
            raise ValueError(f"Channel {s}: lower boundary must be less than upper")
        margin = j.get("margin_deg")
        if margin is not None and not (0 <= valid_number(margin, "margin") <= 30):
            raise ValueError("Margin must be 0..30 degrees")
        neutral = j.get("neutral_command_deg")
        if neutral is not None and not DRIVER_BOUNDS[0] <= valid_number(neutral, "neutral") <= DRIVER_BOUNDS[1]:
            raise ValueError("Neutral outside original driver bounds")
        if len(j.get("speed_observations", [])) > 10000:
            raise ValueError("Too many observations")
    return doc

def load(path):
    if not path.exists():
        raise ValueError(f"No measurement file at {path}. Run 'init' first.")
    return validate(json.loads(path.read_text(encoding="utf-8")))

def save(path, doc, overwrite=True):
    validate(doc)
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        if not overwrite:
            raise ValueError(f"Refusing to overwrite {path}")
        shutil = __import__("shutil")
        shutil.copy2(path, path.with_name(path.name + ".bak"))
    temp = path.with_name(path.name + ".tmp")
    try:
        temp.write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8")
        os.replace(temp, path)
    finally:
        temp.unlink(missing_ok=True)

def safe_bounds(j):
    lo, hi, m = j["lower_observed_deg"], j["upper_observed_deg"], j["margin_deg"]
    if lo is None or hi is None or m is None:
        return None
    if lo + m >= hi - m:
        return None
    return [round(lo+m, 3), round(hi-m, 3)]

def print_table(doc):
    print("OBSERVATION LOG ONLY — no GPIO, I2C or motor commands")
    print("leg joint channel corner         observed limits   margin  usable window  speeds")
    for leg, channels in enumerate(CHANNELS):
        for idx, ch in enumerate(channels):
            j = doc["joints"][str(ch)]
            lo, hi = j["lower_observed_deg"], j["upper_observed_deg"]
            obs = "—" if lo is None or hi is None else f"{lo:g}..{hi:g}"
            bounds = safe_bounds(j)
            usable = "—" if bounds is None else f"{bounds[0]:g}..{bounds[1]:g}"
            margin = "—" if j["margin_deg"] is None else f"{j['margin_deg']:g}"
            print(f"{leg:>3} {idx:>5} {ch:>7} {(j['physical_corner'] or 'unverified'):<14} "
                  f"{obs:<17} {margin:<7} {usable:<14} {len(j['speed_observations']):>3}")

def record_limit(doc, ch, lower, upper, margin):
    check_channel(ch)
    lower = valid_number(lower, "lower")
    upper = valid_number(upper, "upper")
    margin = valid_number(margin, "margin")
    if not (DRIVER_BOUNDS[0] <= lower < upper <= DRIVER_BOUNDS[1]):
        raise ValueError("Observed command limits must obey 18 <= lower < upper <= 162")
    if not (0 < margin <= 30) or lower + margin >= upper - margin:
        raise ValueError("Positive margin must leave a usable window")
    j = doc["joints"][str(ch)]
    j["lower_observed_deg"], j["upper_observed_deg"], j["margin_deg"] = lower, upper, margin
    return safe_bounds(j)

def record_speed(doc, ch, start, end, elapsed, notes=""):
    check_channel(ch)
    start, end, elapsed = (valid_number(x, n) for x, n in
                            ((start,"start"),(end,"end"),(elapsed,"elapsed")))
    if not (DRIVER_BOUNDS[0] <= start <= DRIVER_BOUNDS[1] and
            DRIVER_BOUNDS[0] <= end <= DRIVER_BOUNDS[1]):
        raise ValueError("Command angles outside original servo range")
    if abs(end - start) < 0.1 or elapsed <= 0:
        raise ValueError("Need nonzero travel and positive observed elapsed time")
    speed = abs(end - start)/elapsed
    record = {
        "timestamp_utc": utc_now(),
        "command_start_deg": start,
        "command_end_deg": end,
        "observed_elapsed_s": elapsed,
        "average_command_travel_deg_per_s": round(speed, 3),
        "method": "manual timing; not encoder-verified angular speed",
        "notes": notes
    }
    doc["joints"][str(ch)]["speed_observations"].append(record)
    return record

def export_model(doc):
    joints = []
    for leg, channels in enumerate(CHANNELS):
        for index, ch in enumerate(channels):
            j = doc["joints"][str(ch)]
            timings = j["speed_observations"]
            entries = [t["average_command_travel_deg_per_s"] for t in timings]
            bounds = safe_bounds(j)
            joints.append({
                "leg_index": leg, "joint_index": index, "channel": ch,
                "physical_corner": j["physical_corner"],
                "driver_angle_to_sim_joint_radians": "UNDETERMINED: requires physical servo zero, axis and sign",
                "usable_command_window_degrees": bounds,
                "command_speed_trials_deg_per_s": entries,
                "notes": j["notes"],
                "status": "observed_command_limits" if bounds is not None else "unmeasured"
            })
    return {
        "schema_version": 1, "robot": "Freenove FNK0050",
        "simulation_import_ready": False,
        "reason": "Recorded commanded-angle estimates are not yet mechanical joint-angle limits or validated speed curves",
        "joints": joints
    }

def main(argv=None):
    p = argparse.ArgumentParser(description="Safe OFFLINE observations journal for 12 Dog servos")
    p.add_argument("--file", type=Path, default=DEFAULT_FILE, help="Separate measurement JSON; NEVER calibration.json")
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("init")
    sub.add_parser("status")
    sp = sub.add_parser("limit", help="Record observed lower/upper commanded angles, not hard-stop autodetection")
    sp.add_argument("channel", type=int)
    sp.add_argument("lower_deg", type=float)
    sp.add_argument("upper_deg", type=float)
    sp.add_argument("--margin", type=float, default=5.0,
                    help="Conservative buffer inside observed endpoints (default 5deg)")
    sp = sub.add_parser("speed", help="Record human-timed travel at a given unloaded/load condition")
    sp.add_argument("channel", type=int)
    sp.add_argument("start_deg", type=float)
    sp.add_argument("end_deg", type=float)
    sp.add_argument("elapsed_seconds", type=float)
    sp.add_argument("--notes", default="")
    sp = sub.add_parser("identify", help="Add verified physical corner label and notes")
    sp.add_argument("channel", type=int)
    sp.add_argument("corner", choices=["front-left","front-right","rear-left","rear-right","unknown"])
    sp.add_argument("--notes", default="")
    sp = sub.add_parser("neutral", help="Record independently observed neutral servo command")
    sp.add_argument("channel", type=int)
    sp.add_argument("command_degrees", type=float)
    sub.add_parser("export", help="Export conservative bounds as a separate JSON (not a Webots actuator config)")
    args = p.parse_args(argv)
    try:
        if args.cmd == "init":
            save(args.file, blank_record(), overwrite=False)
            print(f"Created {args.file} (observation log only)")
            return 0
        doc = load(args.file)
        if args.cmd == "status":
            print_table(doc)
        elif args.cmd == "limit":
            bounds = record_limit(doc, args.channel, args.lower_deg, args.upper_deg, args.margin)
            save(args.file, doc)
            print(f"Saved human-observed channel {args.channel} endpoint estimates; buffered window {bounds}")
        elif args.cmd == "speed":
            entry = record_speed(doc, args.channel, args.start_deg, args.end_deg,
                                 args.elapsed_seconds, args.notes)
            save(args.file, doc)
            print(f"Saved channel {args.channel} manually timed average = "
                  f"{entry['average_command_travel_deg_per_s']} commanded degrees/s")
        elif args.cmd == "identify":
            check_channel(args.channel)
            j = doc["joints"][str(args.channel)]
            j["physical_corner"] = None if args.corner == "unknown" else args.corner
            j["notes"] = args.notes
            save(args.file, doc)
            print(f"Saved channel {args.channel} identification")
        elif args.cmd == "neutral":
            check_channel(args.channel)
            value = valid_number(args.command_degrees, "neutral")
            if not DRIVER_BOUNDS[0] <= value <= DRIVER_BOUNDS[1]:
                raise ValueError("Neutral command outside original servo driver bounds")
            doc["joints"][str(args.channel)]["neutral_command_deg"] = value
            save(args.file, doc)
            print(f"Saved observed neutral command {value:g} for channel {args.channel}")
        elif args.cmd == "export":
            out = args.file.with_name("webots_joint_characterization_draft.json")
            if out.exists():
                raise ValueError(f"{out} already exists; move aside to export again")
            out.write_text(json.dumps(export_model(doc), indent=2) + "\n")
            print(f"Exported {out} (NOT automatically usable as calibrated Webots joint limits)")
        return 0
    except (ValueError, OSError, json.JSONDecodeError) as exc:
        p.exit(2, f"ERROR: {exc}\n")

if __name__ == "__main__":
    sys.exit(main())
