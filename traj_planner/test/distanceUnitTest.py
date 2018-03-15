import unittest
import numpy as np
from OneRoadTestEnv.distance import Distance


class MyTestCase(unittest.TestCase):
    distance = Distance()
    obs_points = np.array([[[[0, 2], [2, 2], [2, 0], [0, 0]],
                            [[0, 2], [2, 2], [2, 0], [0, 0]],
                            [[0, 2], [2, 2], [2, 0], [0, 0]]]])
    veh_points = np.array([[10, 1], [10, 2], [10, 3]])

    def test_pseudo_distance_input(self):
        self.assertGreater(self.obs_points[:, :, :, 0].shape, (1, 1, 2),
                           'wrong size in timestamp obstacles positions')
        self.assertEqual(self.obs_points[0, 0, 0, :].shape, (2,),
                         'wrong size in one obstacles positions')

        self.assertGreater(self.veh_points[:, 0].shape, (0,),
                           'wrong size in timestamp egovehicle positions')

        self.assertEqual(self.veh_points[0, :].shape, (2,),
                         'wrong size in one egovehicle positions')

    def test_pseudo_distance_output(self):
        distance_value = self.distance.compute_distance(self.veh_points, self.obs_points)
        self.assertAlmostEqual(distance_value[0][0], 8,
                               2, 'incorrect distance value')
        self.assertAlmostEqual(distance_value[0][1], 8.06,
                               2, 'incorrect distance value')
        self.assertAlmostEqual(distance_value[0][2], 8.246,
                               2, 'incorrect distance value')


def main():
    unittest.main()


if __name__ == '__main__':
    main()
if __name__ == '__main__':
    unittest.main()
