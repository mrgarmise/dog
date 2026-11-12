#!/usr/bin/env python3
"""FNK0050 guarded I2C device interface; import and dry run never access hardware.

Do not use on a robot until electrical testing AND detached/secured servo tests.
A PCA9685 cannot provide real position, stall, or power-cut feedback.
"""
import argparse
import json
import math
import time
from pathlib import Path

CHANNELS = (4, 3, 2, 7, 6, 5, 8, 9, 10, 11, 12, 13)
FREQ = 50
ADDR_SERVO = 0x40
ADDR_IMU = 0x68
CONFIG = Path(__file__).resolve().parent / 'hardware_qualification.json'


def pulse_counts(angle):
    if not (isinstance(angle, (int, float)) and math.isfinite(angle) and 18 <= angle <= 162):
        raise ValueError('angle outside Freenove driver range 18..162 degrees')
    return int((512-102) * angle / 180 + 102)


def check_qualification(path, ch=None, target=None):
    d = json.loads(path.read_text())
    if d.get('qualified_for_servo_power') is not True:
        raise RuntimeError('hardware qualification gate not passed')
    if d.get('manual_hardware_power_cutoff_verified') is not True:
        raise RuntimeError('hardware power cutoff gate not passed')
    if d.get('shield_electrical_checks_passed') is not True:
        raise RuntimeError('shield electrical check gate not passed')
    if ch is not None:
        if ch not in CHANNELS:
            raise ValueError('not a leg servo channel')
        j = d.get('channels', {}).get(str(ch), {})
        lo, hi = j.get('lower_deg'), j.get('upper_deg')
        if not (isinstance(lo, (int,float)) and isinstance(hi,(int,float)) and
                math.isfinite(lo) and math.isfinite(hi) and 18 <= lo < hi <= 162):
            raise RuntimeError(f'channel {ch} has no individually qualified safe window')
        if target is not None and not lo <= target <= hi:
            raise RuntimeError(f'angle {target} outside qualified channel {ch} window {lo}..{hi}')
    return d


class PCA9685:
    """Direct minimal register interface; all I2C imports occur only on construction."""
    def __init__(self, bus_num=1):
        import smbus
        self.bus = smbus.SMBus(bus_num)
        self.closed = False
        try:
            # Do not assume the board or PWM output enable is initially safe.
            # Gate initial physical connection with external motor power DISCONNECTED.
            # Once enabled, immediately program all channels FULL OFF.
            for ch in range(16):
                self.off(ch)
            old = self.bus.read_byte_data(ADDR_SERVO, 0x00)
            self.bus.write_byte_data(ADDR_SERVO, 0x00, (old & 0x7F) | 0x10)
            self.bus.write_byte_data(ADDR_SERVO, 0xFE, 121)  # ~50 Hz
            self.bus.write_byte_data(ADDR_SERVO, 0x00, old & 0xEF)
            time.sleep(0.005)
            self.bus.write_byte_data(ADDR_SERVO, 0x00, (old & 0xEF) | 0x80)
            for ch in range(16):
                self.off(ch)
        except BaseException:
            self.close()
            raise

    def off(self, channel):
        if not 0 <= channel <= 15:
            raise ValueError('channel')
        reg = 0x06 + 4 * channel
        # Explicit full-off bit in LEDn_OFF_H register bit 4.
        self.bus.write_byte_data(ADDR_SERVO, reg, 0)
        self.bus.write_byte_data(ADDR_SERVO, reg+1, 0)
        self.bus.write_byte_data(ADDR_SERVO, reg+2, 0)
        self.bus.write_byte_data(ADDR_SERVO, reg+3, 0x10)

    def set_angle(self, channel, angle):
        n = pulse_counts(angle)
        reg = 0x06 + 4 * channel
        self.bus.write_byte_data(ADDR_SERVO, reg, 0)
        self.bus.write_byte_data(ADDR_SERVO, reg+1, 0)
        self.bus.write_byte_data(ADDR_SERVO, reg+2, n & 0xff)
        self.bus.write_byte_data(ADDR_SERVO, reg+3, (n >> 8) & 0x0f)

    def close(self):
        if self.closed:
            return
        try:
            for ch in range(16):
                try:
                    self.off(ch)
                except OSError:
                    pass
        finally:
            self.closed = True
            self.bus.close()


class MPU6050Tilt:
    """Read raw accelerometer tilt; no yaw, no dynamic sensor fusion.

    Axis assignments must be checked against actual board mounting.
    Stationary tilt only; acceleration while walking affects readings.
    """
    def __init__(self, bus_num=1):
        import smbus
        self.bus = smbus.SMBus(bus_num)
        self.bus.write_byte_data(ADDR_IMU, 0x6B, 0x00)  # wake
        self.bus.write_byte_data(ADDR_IMU, 0x1C, 0x00)  # ±2g
        self.offset_roll = 0.0
        self.offset_pitch = 0.0

    def _i16(self, reg):
        hi = self.bus.read_byte_data(ADDR_IMU, reg)
        lo = self.bus.read_byte_data(ADDR_IMU, reg + 1)
        x = (hi << 8) | lo
        return x - 65536 if x > 32767 else x

    def read(self):
        ax, ay, az = (self._i16(reg) / 16384 for reg in (0x3B, 0x3D, 0x3F))
        magnitude = math.sqrt(ax*ax + ay*ay + az*az)
        if not 0.75 <= magnitude <= 1.25:
            raise RuntimeError(f'gravity magnitude {magnitude:.2f}g outside stationary range')
        roll = math.degrees(math.atan2(ay, az))
        pitch = math.degrees(math.atan2(-ax, math.hypot(ay, az)))
        return roll-self.offset_roll, pitch-self.offset_pitch, magnitude

    def zero(self, samples=40):
        vals=[]
        for _ in range(samples):
            vals.append(self.read())
            time.sleep(0.02)
        self.offset_roll += sum(v[0] for v in vals)/len(vals)
        self.offset_pitch += sum(v[1] for v in vals)/len(vals)

    def close(self):
        self.bus.close()


def main(argv=None):
    ap=argparse.ArgumentParser(description='Dog guarded motor & IMU utility; default DRY RUN')
    ap.add_argument('--live', action='store_true', help='Use real I2C hardware only after qualification')
    ap.add_argument('--ack', choices=['I_HAVE_VERIFIED_ELECTRICAL_AND_MECHANICAL_SAFETY'])
    ap.add_argument('--qualification', type=Path, default=CONFIG)
    sub=ap.add_subparsers(dest='cmd', required=True)
    move=sub.add_parser('jog', help='ONLY one explicit target; no automatic sweep')
    move.add_argument('channel', type=int, choices=CHANNELS)
    move.add_argument('target_deg', type=float)
    move.add_argument('--hold-seconds', type=float, default=0.4)
    tilt=sub.add_parser('tilt', help='Read stationary raw accelerometer level; NO motor movement')
    tilt.add_argument('--seconds', type=float, default=5.0)
    tilt.add_argument('--zero-on-start', action='store_true')
    args=ap.parse_args(argv)
    if args.cmd == 'jog':
        if not 0.1 <= args.hold_seconds <= 2.0:
            ap.error('hold duration must be 0.1..2 sec')
        try:
            count=pulse_counts(args.target_deg)
        except ValueError as e:
            ap.error(str(e))
        if not args.live:
            print(f'DRY RUN: channel {args.channel} target {args.target_deg:.1f} deg, {count} counts; NO I2C')
            return
    if args.cmd == 'tilt' and not 0.1 <= args.seconds <= 60:
        ap.error('tilt duration must be 0.1..60 sec')
    if not args.live:
        if args.cmd == 'tilt':
            print('DRY RUN: MPU6050 stationary tilt readout, no hardware accessed')
        return
    if args.ack != 'I_HAVE_VERIFIED_ELECTRICAL_AND_MECHANICAL_SAFETY':
        ap.error('--live requires --ack I_HAVE_VERIFIED_ELECTRICAL_AND_MECHANICAL_SAFETY')
    try:
        check_qualification(args.qualification, args.channel if args.cmd=='jog' else None,
                            args.target_deg if args.cmd=='jog' else None)
    except (OSError, ValueError, RuntimeError) as e:
        ap.error(f'Qualification locked: {e}')
    if args.cmd == 'jog':
        print('WARNING: FIRST PWM COMMAND MAY JUMP TO TARGET. Physically isolate or decouple servo.')
        print('Hardware must be safely restrained; external cutoff supervised. Ctrl-C is NOT power cutoff.')
        dev = PCA9685()
        try:
            dev.set_angle(args.channel, args.target_deg)
            time.sleep(args.hold_seconds)
        finally:
            dev.close()
        print('PWM off for all channels; servo power may still be on.')
    else:
        sensor=MPU6050Tilt()
        try:
            if args.zero_on_start:
                print('Zeroing tilt from stationary current pose — this does NOT imply level')
                sensor.zero()
            until=time.monotonic()+args.seconds
            while time.monotonic()<until:
                r,p,g=sensor.read()
                print(f'roll={r:+7.2f}° pitch={p:+7.2f}° magnitude={g:.3f}g')
                time.sleep(.2)
        finally:
            sensor.close()

if __name__=='__main__':
    main()
