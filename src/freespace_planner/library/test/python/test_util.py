import unittest
import math
from path_planner_lib.util import constrain_angle_minpi_pluspi, angle_diff, constrain_angle_minpi_pluspi_fmod


class UtilsTest(unittest.TestCase):
    def test_constrain_angle_minpi_pluspi(self):
        # lower limit
        angle: float = -math.pi
        offset: float = -0.01
        angle2check: float = angle + offset
        result: float = constrain_angle_minpi_pluspi(angle2check)
        result2: float = constrain_angle_minpi_pluspi_fmod(angle2check)
        self.assertGreater(result, -math.pi)
        self.assertLess(result, math.pi)
        self.assertGreater(result2, -math.pi)
        self.assertLess(result2, math.pi)

        # upper limit
        angle: float = math.pi
        offset: float = +0.01
        angle2check: float = angle + offset
        result: float = constrain_angle_minpi_pluspi(angle2check)
        result2: float = constrain_angle_minpi_pluspi_fmod(angle2check)
        self.assertGreater(result, -math.pi)
        self.assertLess(result, math.pi)
        self.assertGreater(result2, -math.pi)
        self.assertLess(result2, math.pi)

        # inside negative
        angle: float = -math.pi
        offset: float = +0.01
        angle2check: float = angle + offset
        result: float = constrain_angle_minpi_pluspi(angle2check)
        result2: float = constrain_angle_minpi_pluspi_fmod(angle2check)
        self.assertEqual(result, angle2check)
        self.assertEqual(result2, angle2check)

        # inside positive
        angle: float = math.pi
        offset: float = -0.01
        angle2check: float = angle + offset
        result: float = constrain_angle_minpi_pluspi(angle2check)
        result2: float = constrain_angle_minpi_pluspi_fmod(angle2check)
        self.assertEqual(result, angle2check)
        self.assertEqual(result2, angle2check)

        # inside very negative
        angle: float = -99 * math.pi
        offset: float = +0.01
        angle2check: float = angle + offset
        print("angle2check", angle2check)
        result: float = constrain_angle_minpi_pluspi(angle2check)
        result2: float = constrain_angle_minpi_pluspi_fmod(angle2check)
        print("result", result)
        print("result2", result2)
        exp_result: float = -math.pi + offset
        self.assertAlmostEqual(result, exp_result)
        self.assertAlmostEqual(result2, exp_result)

        # inside very positive
        angle: float = 99 * math.pi
        offset: float = -0.01
        angle2check: float = angle + offset
        result: float = constrain_angle_minpi_pluspi(angle2check)
        result2: float = constrain_angle_minpi_pluspi_fmod(angle2check)
        exp_result: float = math.pi + offset
        self.assertAlmostEqual(result, exp_result)
        self.assertAlmostEqual(result2, exp_result)

        # boundary cases
        angle: float = 2 * math.pi
        offset: float = 0
        angle2check: float = angle + offset
        result: float = constrain_angle_minpi_pluspi(angle2check)
        result2: float = constrain_angle_minpi_pluspi_fmod(angle2check)
        exp_result: float = 0.0
        self.assertAlmostEqual(result, exp_result)
        self.assertAlmostEqual(result2, exp_result)

        # pi is shifted to min pi
        angle: float = math.pi
        offset: float = 0
        angle2check: float = angle + offset
        result: float = constrain_angle_minpi_pluspi(angle2check)
        result2: float = constrain_angle_minpi_pluspi_fmod(angle2check)
        exp_result: float = -math.pi
        self.assertAlmostEqual(result, exp_result)
        self.assertAlmostEqual(result2, exp_result)

        # pi is shifted to min pi
        angle: float = -math.pi
        offset: float = 0
        angle2check: float = angle + offset
        result: float = constrain_angle_minpi_pluspi(angle2check)
        result2: float = constrain_angle_minpi_pluspi_fmod(angle2check)
        exp_result: float = -math.pi
        self.assertAlmostEqual(result, exp_result)
        self.assertAlmostEqual(result2, exp_result)

    def test_angle_diff(self):

        angle1: float = 0
        angle2: float = math.pi
        result: float = angle_diff(angle1, angle2)
        expected_result: float = -math.pi
        self.assertAlmostEqual(result, expected_result)

        angle1: float = 2
        angle2: float = 0
        result: float = angle_diff(angle1, angle2)
        expected_result: float = -2
        self.assertAlmostEqual(result, expected_result)

        angle1: float = 0
        angle2: float = 2
        result: float = angle_diff(angle1, angle2)
        expected_result: float = 2
        self.assertAlmostEqual(result, expected_result)

        angle1: float = math.pi/2
        angle2: float = -math.pi/2
        result: float = angle_diff(angle1, angle2)
        expected_result: float = -math.pi
        self.assertAlmostEqual(result, expected_result)

        angle1: float = math.pi-1
        angle2: float = -math.pi+1
        result: float = angle_diff(angle1, angle2)
        expected_result: float = 2
        self.assertAlmostEqual(result, expected_result)

        angle1: float = -math.pi+1
        angle2: float = math.pi-1
        result: float = angle_diff(angle1, angle2)
        expected_result: float = -2
        self.assertAlmostEqual(result, expected_result)

