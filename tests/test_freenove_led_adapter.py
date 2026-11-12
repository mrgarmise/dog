import unittest
from dog_runtime.balance_compass import BalanceCompass
from dog_runtime.freenove_led_adapter import FreenoveCompassOutput


class FakeStrip:
    def __init__(self):
        self.values = [(0, 0, 0)] * 7
        self.shows = 0
    def get_led_count(self):
        return 7
    def set_led_rgb_data(self, index, rgb):
        self.values[index] = tuple(rgb)
    def show(self):
        self.shows += 1


class FakeLed:
    is_support_led_function = True
    def __init__(self):
        self.strip = FakeStrip()


class HardwareAdapterTests(unittest.TestCase):
    def test_seven_leds_and_permission(self):
        led = FakeLed()
        adapter = FreenoveCompassOutput(led)
        frame = BalanceCompass().frame(0.0, roll=0, pitch=0)
        self.assertFalse(adapter.write(frame, now=0))
        self.assertEqual(led.strip.shows, 0)
        self.assertTrue(adapter.write(frame, now=0, permission=True))
        self.assertEqual(led.strip.values, list(frame.rgb))
        self.assertFalse(adapter.write(frame, now=.01, permission=True))
        self.assertFalse(adapter.write(frame, now=.5, permission=True, fault_active=True))
        self.assertEqual(led.strip.shows, 1)
    def test_reject_wrong_count(self):
        led = FakeLed()
        led.strip.get_led_count = lambda: 8
        with self.assertRaises(RuntimeError):
            FreenoveCompassOutput(led)


if __name__ == "__main__":
    unittest.main()
