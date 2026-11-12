"""Freenove LED adapter for Balance Compass.

No hardware setup is performed here. Caller owns Led() initialization and
must arbitrate other LED users (fault indicators take priority).
"""
import time


class FreenoveCompassOutput:
    def __init__(self, led, *, max_hz=20.0):
        if not 0 < max_hz <= 50:
            raise ValueError("invalid update rate")
        self.led = led
        self.min_period = 1.0 / max_hz
        self.last_write = None
        if not getattr(led, "is_support_led_function", False):
            raise RuntimeError("Freenove LED driver not supported on this board")
        self.strip = led.strip
        if self.strip.get_led_count() != 7:
            raise RuntimeError("Expected exactly 7 physical LEDs")

    def write(self, frame, *, permission=False, fault_active=False, now=None):
        """Write only with explicit ownership; never override a fault indication."""
        if not permission or fault_active:
            return False
        now = time.monotonic() if now is None else float(now)
        if self.last_write is not None and now - self.last_write < self.min_period:
            return False
        if len(frame.rgb) != 7:
            raise ValueError("Expected seven RGB values")
        for i, rgb in enumerate(frame.rgb):
            if len(rgb) != 3 or any(not isinstance(c, int) or c < 0 or c > 60 for c in rgb):
                raise ValueError("invalid brightness")
            self.strip.set_led_rgb_data(i, list(rgb))
        self.strip.show()
        self.last_write = now
        return True
