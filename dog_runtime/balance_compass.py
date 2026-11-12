"""Shared FNK0050 Balance Compass logic. No GPIO or motor I/O.

Body coordinates: +X forward, +Y left, +Z up. Logical LEDs:
front, front-left, left, rear-left, rear, rear-right, right, front-right.
"""
from dataclasses import dataclass
import math

@dataclass(frozen=True)
class CompassFrame:
    mode: str
    rgb: tuple
    vector: tuple
    qualified: bool

def _clamp(v, low, high):
    return min(high, max(low, v))

def tilt_vector(roll, pitch, full_scale=0.35):
    """Gravity downhill direction in body XY, normalized to full_scale."""
    if not all(map(math.isfinite, (roll, pitch, full_scale))) or not 0 < full_scale < math.pi / 2:
        raise ValueError("invalid attitude")
    d = math.sin(full_scale)
    return (_clamp(math.sin(pitch) / d, -1, 1),
            _clamp(-math.sin(roll) * math.cos(pitch) / d, -1, 1))

def eight_led_rgb(vector, color, base=0.14, contrast=0.12, cap=60):
    """Balanced means all eight equally dim; offset makes near LEDs brighter."""
    if len(vector) != 2 or color not in ("red", "green"):
        raise ValueError("invalid vector or color")
    x, y = map(float, vector)
    if not all(map(math.isfinite, (x, y, base, contrast))):
        raise ValueError("nonfinite value")
    if not 0 < contrast < base < 1 - contrast or not 1 <= cap <= 255:
        raise ValueError("invalid intensity limits")
    out = []
    for i in range(8):
        direction = i * math.pi / 4
        projected = _clamp(x * math.cos(direction) + y * math.sin(direction), -1, 1)
        value = max(1, round(cap * (base + contrast * projected)))
        out.append((value, 0, 0) if color == "red" else (0, value, 0))
    return tuple(out)

class BalanceCompass:
    """Two 0.5-second alternating modes. COM must be supplied by an estimator."""
    def __init__(self, half_cycle=0.5, led_order=None):
        if not math.isfinite(half_cycle) or half_cycle <= 0:
            raise ValueError("invalid half-cycle")
        self.half_cycle = half_cycle
        self.led_order = tuple(range(8)) if led_order is None else tuple(led_order)
        if sorted(self.led_order) != list(range(8)):
            raise ValueError("led_order must be a permutation of 0..7")

    def frame(self, elapsed_s, *, roll, pitch, com_vector=None, com_valid=False):
        if not math.isfinite(elapsed_s) or elapsed_s < 0:
            raise ValueError("invalid elapsed time")
        mode = "red-tilt" if int(elapsed_s / self.half_cycle) % 2 == 0 else "green-com"
        if mode == "red-tilt":
            vector = tilt_vector(roll, pitch)
            colors = eight_led_rgb(vector, "red")
            qualified = True
        elif com_valid and com_vector is not None:
            vector = tuple(map(float, com_vector))
            colors = eight_led_rgb(vector, "green")
            qualified = True
        else:
            # Never show all-dim green ('balanced') if COM cannot be estimated.
            vector = (0.0, 0.0)
            colors = tuple((0, 3 if i % 2 == 0 else 0, 0) for i in range(8))
            qualified = False
        physical = [(0, 0, 0)] * 8
        for i, physical_index in enumerate(self.led_order):
            physical[physical_index] = colors[i]
        return CompassFrame(mode, tuple(physical), vector, qualified)
