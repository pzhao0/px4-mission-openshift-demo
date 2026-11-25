"""Check that the chosen point follows the box midpoint on the inspection face."""
from pathlib import Path
import sys
import unittest

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "applications/runtime"))
from stop_sign import OCTAGON, inspection_point


class InspectionPointTest(unittest.TestCase):
    def setUp(self):
        self.camera = np.float64([[135, 0, 160], [0, 135, 120], [0, 0, 1]])
        self.frame = np.zeros((240, 320, 3), dtype=np.uint8)
        pixels, _ = cv2.projectPoints(OCTAGON, np.zeros(3), np.float64([0, 0, 5]),
                                      self.camera, None)
        cv2.fillConvexPoly(self.frame, pixels.reshape(-1, 2).round().astype(np.int32),
                           (0, 0, 255))

    def test_shifted_box_aims_at_its_midpoint(self):
        for box in ([140, 100, 180, 140], [140, 100, 184, 142]):
            with self.subTest(box=box):
                point = inspection_point(self.frame, box, self.camera)
                self.assertIsNotNone(point)
                projection = self.camera @ point
                midpoint = (np.array(box[:2]) + box[2:]) / 2
                np.testing.assert_allclose(projection[:2] / projection[2], midpoint, atol=.01)

    def test_empty_or_clipped_face(self):
        self.assertIsNone(inspection_point(self.frame, [0, 100, 180, 140], self.camera))
        self.assertIsNone(inspection_point(np.zeros_like(self.frame), [140, 100, 180, 140],
                                           self.camera))


if __name__ == "__main__":
    unittest.main()
