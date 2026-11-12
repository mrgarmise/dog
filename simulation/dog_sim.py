#!/usr/bin/env python3
"""Freenove FNK0050 *offline* servo/IK simulator. No hardware interface exists."""
import argparse
import json
import math
from pathlib import Path

DEFAULT_CONFIG = Path(__file__).resolve().parent / 'calibration.json'
CHANNELS = ((4, 3, 2), (7, 6, 5), (8, 9, 10), (11, 12, 13))
NOMINAL = ((0, 99, 10), (0, 99, 10), (0, 99, -10), (0, 99, -10))
LEG_LINKS = (23, 55, 55)  # mm, sourced from Freenove Control.py
SERVO_MIN, SERVO_MAX = 18, 162  # Freenove Servo.py driver clamp


def starting_config():
    return {
        'schema_version': 1,
        'device': 'Freenove FNK0050',
        'mode': 'simulation_only',
        'servo_limits_degrees': [SERVO_MIN, SERVO_MAX],
        'nominal_foot_coordinates_mm': [list(x) for x in NOMINAL],
        'calibration_foot_coordinates_mm': [list(x) for x in NOMINAL],
        'notes': 'Leg order follows Control.py indexes 0-3. Physical corners not yet verified.',
    }


def validate_points(points, label):
    if not isinstance(points, list) or len(points) != 4:
        raise ValueError(f'{label} must contain exactly four legs')
    for i, point in enumerate(points):
        if not isinstance(point, list) or len(point) != 3:
            raise ValueError(f'{label}[{i}] must have x, y, z')
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in point):
            raise ValueError(f'{label}[{i}] must contain finite numbers')
        ik(*point)  # validates full mathematical reachability


def validate_config(cfg):
    if not isinstance(cfg, dict) or cfg.get('schema_version') != 1 or cfg.get('mode') != 'simulation_only':
        raise ValueError('Not a recognized simulation-only calibration file')
    if cfg.get('servo_limits_degrees') != [SERVO_MIN, SERVO_MAX]:
        raise ValueError('Servo limits cannot be changed in this simulation version')
    validate_points(cfg.get('nominal_foot_coordinates_mm'), 'nominal_foot_coordinates_mm')
    validate_points(cfg.get('calibration_foot_coordinates_mm'), 'calibration_foot_coordinates_mm')
    return cfg


def load_config(path):
    if not path.exists():
        return starting_config()
    return validate_config(json.loads(path.read_text(encoding='utf-8')))


def save_config(path, cfg):
    validate_config(cfg)
    path.parent.mkdir(parents=True, exist_ok=True)
    # Never overwrite an existing config directly without retaining a backup.
    if path.exists():
        import shutil
        shutil.copy2(path, path.with_name(path.name + '.bak'))
    temp = path.with_name(path.name + '.tmp')
    try:
        temp.write_text(json.dumps(cfg, indent=2) + '\n', encoding='utf-8')
        temp.replace(path)
    finally:
        temp.unlink(missing_ok=True)


def ik(x, y, z, l1=23, l2=55, l3=55):
    """Freenove coordinateToAngle formula, with errors instead of silent failures."""
    if any(not math.isfinite(t) for t in (x, y, z)):
        raise ValueError('Non-finite coordinate')
    if not 25 <= math.sqrt(x*x + y*y + z*z) <= 130:
        raise ValueError('Outside Freenove foot-distance guard (25..130 mm)')
    a = math.pi/2 - math.atan2(z, y)
    x4, x5 = l1 * math.sin(a), l1 * math.cos(a)
    l23 = math.sqrt((z-x5)**2+(y-x4)**2+x*x)
    if l23 < 1e-9:
        raise ValueError('Singular leg geometry')
    w = x / l23
    v = (l2*l2 + l23*l23 - l3*l3) / (2*l2*l23)
    k = (l2*l2 + l3*l3 - l23*l23) / (2*l3*l2)
    # The stock formula rounds trig arguments to 2dp. Preserve that behavior
    # only for valid values; reject unreachable targets rather than clipping.
    if any(abs(t) > 1 + 1e-9 for t in (w, v, k)):
        raise ValueError('Unreachable foot coordinates (IK domain)')
    b = math.asin(round(max(-1, min(1, w)), 2)) - math.acos(round(max(-1, min(1, v)), 2))
    c = math.pi - math.acos(round(max(-1, min(1, k)), 2))
    return tuple(round(math.degrees(t)) for t in (a, b, c))


def joint_targets(cfg, feet):
    validate_config(cfg)
    validate_points(feet, 'requested feet')
    # Freenove calibration: IK(calibration foot) - IK(nominal foot)
    adjustments = []
    for cal, nominal in zip(cfg['calibration_foot_coordinates_mm'], cfg['nominal_foot_coordinates_mm']):
        adjustments.append(tuple(c - n for c, n in zip(ik(*cal), ik(*nominal))))
    rows = []
    for i, (foot, offset) in enumerate(zip(feet, adjustments)):
        a, b, c = ik(*foot)
        da, db, dc = offset
        if i < 2:
            angles = (a + da, 90 - (b + db), c + dc)
        else:
            angles = (a + da, 90 + b + db, 180 - (c + dc))
        for j, (channel, requested) in enumerate(zip(CHANNELS[i], angles)):
            within = SERVO_MIN <= requested <= SERVO_MAX
            actual = min(SERVO_MAX, max(SERVO_MIN, requested))
            pwm = int((512 - 102) * actual / 180 + 102)
            rows.append({
                'leg_index': i, 'joint_index': j, 'channel': channel,
                'requested_angle_deg': requested,
                'driver_clamped_angle_deg': actual,
                'pwm_count_at_50hz': pwm,
                'within_driver_limits': within,
            })
    return rows


def parse_triplets(text):
    """Four semicolon-separated x,y,z triplets, e.g. '0,99,10;...'"""
    legs = []
    for triplet in text.split(';'):
        values = triplet.split(',')
        if len(values) != 3:
            raise ValueError('Each leg requires x,y,z')
        legs.append([float(t.strip()) for t in values])
    validate_points(legs, 'requested feet')
    return legs


def read_freenove_point(path):
    lines = path.read_text(encoding='utf-8').splitlines()
    legs = [[float(p) for p in line.split()] for line in lines if line.strip()]
    validate_points(legs, 'Freenove point.txt')
    return legs


def main():
    p = argparse.ArgumentParser(description='OFFLINE Dog servo simulator — never touches GPIO, I2C, or motors')
    p.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    sub = p.add_subparsers(dest='command', required=True)
    sub.add_parser('init', help='Create calibration.json (refuses to overwrite)')
    pose = sub.add_parser('pose', help='Display simulated servo angles and PWM counts')
    pose.add_argument('--feet', help='Four x,y,z triplets separated with semicolons; default nominal standing pose')
    setcal = sub.add_parser('set-cal', help='Save a calibration foot point for one indexed leg')
    setcal.add_argument('leg', type=int, choices=range(4))
    setcal.add_argument('x', type=float)
    setcal.add_argument('y', type=float)
    setcal.add_argument('z', type=float)
    imp = sub.add_parser('import-point', help='Import Freenove point.txt as calibration coordinates ONLY; no hardware')
    imp.add_argument('path', type=Path)
    sub.add_parser('show-config', help='Print calibration file, or unsaved defaults')
    a = p.parse_args()
    try:
        cfg = load_config(a.config)
        if a.command == 'init':
            if a.config.exists():
                raise ValueError(f'{a.config} already exists (refusing to overwrite)')
            save_config(a.config, cfg)
            print('Created', a.config)
        elif a.command == 'show-config':
            print(json.dumps(cfg, indent=2))
        elif a.command == 'pose':
            feet = parse_triplets(a.feet) if a.feet else cfg['nominal_foot_coordinates_mm']
            rows = joint_targets(cfg, feet)
            print('SIMULATION ONLY — no I2C/GPIO/motor operations')
            print('leg joint channel requested° clamped° PWM@50Hz status')
            for r in rows:
                status = 'OK' if r['within_driver_limits'] else 'LIMIT (stock driver would clamp)'
                print(f"{r['leg_index']:>3} {r['joint_index']:>5} {r['channel']:>7} "
                      f"{r['requested_angle_deg']:>9.0f} {r['driver_clamped_angle_deg']:>8.0f} "
                      f"{r['pwm_count_at_50hz']:>8} {status}")
            if not all(r['within_driver_limits'] for r in rows):
                print('WARNING: One or more targets exceed stock Servo.py driver limits.')
        elif a.command == 'set-cal':
            cfg['calibration_foot_coordinates_mm'][a.leg] = [a.x, a.y, a.z]
            validate_config(cfg)
            save_config(a.config, cfg)
            print(f'Saved simulated calibration point for leg {a.leg} to {a.config}')
        elif a.command == 'import-point':
            cfg['calibration_foot_coordinates_mm'] = read_freenove_point(a.path)
            save_config(a.config, cfg)
            print(f'Imported calibration coordinates to {a.config} (original source untouched)')
    except (ValueError, OSError, json.JSONDecodeError) as exc:
        p.exit(2, f'ERROR: {exc}\n')


if __name__ == '__main__':
    main()
