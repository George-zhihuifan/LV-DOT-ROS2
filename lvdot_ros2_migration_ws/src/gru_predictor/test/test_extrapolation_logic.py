import unittest
import numpy as np


def extrapolate(hybrid_pred, current_pos, horizon):
    vel = hybrid_pred - current_pos
    future_positions = [hybrid_pred.copy()]
    for _ in range(1, horizon):
        future_positions.append((future_positions[-1] + vel).copy())
    return future_positions


class TestExtrapolation(unittest.TestCase):
    def test_non_repeating_positions(self):
        hp = np.array([2.0, 2.0, 2.0], dtype=float)
        cp = np.array([1.0, 1.0, 1.0], dtype=float)
        out = extrapolate(hp, cp, 4)
        self.assertEqual(len(out), 4)
        self.assertTrue((out[1] != out[0]).any())
        self.assertTrue((out[2] != out[1]).any())


if __name__ == '__main__':
    unittest.main()
