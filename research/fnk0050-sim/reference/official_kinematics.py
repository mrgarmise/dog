# Extracted unchanged methods from Freenove Control.py; CC BY-NC-SA 3.0.
import math

class Official:

    def coordinateToAngle(self, x, y, z, l1=23, l2=55, l3=55):
        a = math.pi / 2 - math.atan2(z, y)
        x_3 = 0
        x_4 = l1 * math.sin(a)
        x_5 = l1 * math.cos(a)
        l23 = math.sqrt((z - x_5) ** 2 + (y - x_4) ** 2 + (x - x_3) ** 2)
        w = (x - x_3) / l23
        v = (l2 * l2 + l23 * l23 - l3 * l3) / (2 * l2 * l23)
        b = math.asin(round(w, 2)) - math.acos(round(v, 2))
        c = math.pi - math.acos(round((l2 ** 2 + l3 ** 2 - l23 ** 2) / (2 * l3 * l2), 2))
        a = round(math.degrees(a))
        b = round(math.degrees(b))
        c = round(math.degrees(c))
        return (a, b, c)

    def angleToCoordinate(self, a, b, c, l1=23, l2=55, l3=55):
        a = math.pi / 180 * a
        b = math.pi / 180 * b
        c = math.pi / 180 * c
        x = l3 * math.sin(b + c) + l2 * math.sin(b)
        y = l3 * math.sin(a) * math.cos(b + c) + l2 * math.sin(a) * math.cos(b) + l1 * math.sin(a)
        z = l3 * math.cos(a) * math.cos(b + c) + l2 * math.cos(a) * math.cos(b) + l1 * math.cos(a)
        return (x, y, z)
