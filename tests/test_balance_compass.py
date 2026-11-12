import math
import unittest
from dog_runtime.balance_compass import BalanceCompass, tilt_vector, seven_led_rgb

class BalanceCompassTests(unittest.TestCase):
    def test_neutral_all_dim(self):
        for color in ("red", "green"):
            colors = seven_led_rgb((0,0), color)
            self.assertEqual(len(colors), 7)
            self.assertEqual(len(set(colors)), 1)
            self.assertTrue(any(colors[0]))
    def test_front_down_bright_red(self):
        c = seven_led_rgb(tilt_vector(0, .15), "red")
        self.assertGreater(c[0][0], c[3][0])
        self.assertGreater(c[4][0], 0)
    def test_right_down_bright_red(self):
        c = seven_led_rgb(tilt_vector(.15, 0), "red")
        self.assertGreater(c[5][0], c[2][0])
    def test_flip_half_second(self):
        compass = BalanceCompass()
        a = compass.frame(.49, roll=0, pitch=0, com_vector=(0,0), com_valid=True)
        b = compass.frame(.50, roll=0, pitch=0, com_vector=(0,0), com_valid=True)
        self.assertEqual(a.mode, "red-tilt")
        self.assertEqual(b.mode, "green-com")
        self.assertEqual(len(set(b.rgb)), 1)
    def test_missing_com_not_marked_balanced(self):
        c = BalanceCompass().frame(.6, roll=0, pitch=0)
        self.assertFalse(c.qualified)
        self.assertNotEqual(len(set(c.rgb)), 1)
    def test_led_reorder(self):
        c = BalanceCompass(led_order=list(reversed(range(7))))
        f = c.frame(0, roll=0, pitch=.2)
        self.assertGreater(f.rgb[6][0], f.rgb[3][0])
    def test_reject_bad_input(self):
        with self.assertRaises(ValueError):
            tilt_vector(math.nan, 0)
        with self.assertRaises(ValueError):
            BalanceCompass(led_order=[0]*7)

if __name__ == "__main__":
    unittest.main()
