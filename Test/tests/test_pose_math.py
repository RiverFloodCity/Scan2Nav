import math
import unittest

from src.navigation.pose_utils import normalize_yaw, quaternion_wxyz_to_yaw


def yaw_to_quaternion(yaw: float):
    return (
        math.cos(yaw / 2.0),
        0.0,
        0.0,
        math.sin(yaw / 2.0),
    )


class PoseMathTests(unittest.TestCase):
    def assertYawAlmostEqual(self, actual, expected):
        self.assertAlmostEqual(normalize_yaw(actual - expected), 0.0)

    def test_yaw_zero(self):
        self.assertYawAlmostEqual(
            quaternion_wxyz_to_yaw(*yaw_to_quaternion(0.0)),
            0.0,
        )

    def test_yaw_pi_over_two(self):
        self.assertYawAlmostEqual(
            quaternion_wxyz_to_yaw(*yaw_to_quaternion(math.pi / 2.0)),
            math.pi / 2.0,
        )

    def test_yaw_negative_pi_over_two(self):
        self.assertYawAlmostEqual(
            quaternion_wxyz_to_yaw(*yaw_to_quaternion(-math.pi / 2.0)),
            -math.pi / 2.0,
        )

    def test_yaw_close_to_pi(self):
        yaw = math.pi - 1e-9
        self.assertYawAlmostEqual(
            quaternion_wxyz_to_yaw(*yaw_to_quaternion(yaw)),
            yaw,
        )

    def test_normalize_yaw_above_range(self):
        self.assertYawAlmostEqual(
            normalize_yaw(5.0 * math.pi / 2.0),
            math.pi / 2.0,
        )

    def test_normalize_yaw_below_range(self):
        self.assertYawAlmostEqual(
            normalize_yaw(-5.0 * math.pi / 2.0),
            -math.pi / 2.0,
        )

    def test_pi_normalizes_to_negative_pi(self):
        self.assertEqual(normalize_yaw(math.pi), -math.pi)


if __name__ == "__main__":
    unittest.main()
