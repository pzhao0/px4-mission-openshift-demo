"""Check capture-time matching, trail gaps and replay isolation without ROS."""
import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'applications/dashboard'))
from survey_view import SurveyView


class DashboardSurveyTest(unittest.TestCase):
    def setUp(self):
        self.view = SurveyView()
        self.event('search_searching', 1)

    def event(self, phase, run, sighting=0, hold=0):
        self.view.status({'phase': phase, 'survey': {
            'run_ns': run, 'sighting_ns': sighting, 'hold_frame_ns': hold,
        }})

    def test_capture_before_or_after_sighting_event(self):
        for event_first in (False, True):
            with self.subTest(event_first=event_first):
                self.setUp()
                v = self.view
                stamp = 1_050_000_000
                v.position(1_000_000_000, 2, 22, 5)
                v.position(1_100_000_000, 3, 22, 5)
                if event_first:
                    self.event('search_spotted', 1, stamp)
                v.preview(stamp, b'accepted-frame')
                v.preview(stamp + 10, b'next-frame')
                self.event('search_spotted', 1, stamp)
                self.assertEqual(v.first_image, b'accepted-frame')
                self.assertEqual(v.first_en, [2.5, 22])
                self.event('ready', 1, stamp)
                self.assertTrue(v.snapshot()['has_image'])
                self.event('demo_takeoff', 2)
                self.assertFalse(v.snapshot()['has_image'])
                self.assertEqual(v.trail, [])

    def test_pinned_photo_matches_hold_frame_in_either_delivery_order(self):
        for preview_first in (False, True):
            with self.subTest(preview_first=preview_first):
                self.setUp()
                v = self.view
                self.event('search_spotted', 1, sighting=10)
                v.preview(10, b'approach')
                self.assertEqual(v.snapshot()['hold_photo_ns'], 0)
                if preview_first:
                    v.preview(20, b'centered')
                self.event('search_investigating', 1, sighting=10, hold=20)
                if not preview_first:
                    self.assertIsNone(v.hold_image)
                    v.preview(20, b'centered')
                v.preview(21, b'next live frame')
                self.assertEqual(v.hold_image, b'centered')
                self.assertEqual(v.snapshot()['hold_photo_ns'], 20)
                self.event('search_spotted', 1, sighting=10)
                self.assertIsNone(v.hold_image)
                self.event('search_investigating', 1, sighting=10, hold=30)
                v.preview(30, b'retry')
                self.event('ready', 1, sighting=10, hold=30)
                self.assertEqual(v.hold_image, b'retry')
                self.event('demo_takeoff', 2)
                self.assertEqual(v.snapshot()['hold_photo_ns'], 0)
                self.assertIsNone(v.hold_image)

    def test_map_keeps_only_the_successful_hold(self):
        v = self.view
        v.position(1_000_000_000, 2, 22, 5)
        self.event('search_investigating', 1)
        v.position(2_000_000_000, 7, 22, 5)
        self.assertIn(None, v.trail)
        self.assertEqual(v.hold_en, [7, 22])
        self.event('search_searching', 1)
        self.assertIsNone(v.hold_en)
        self.event('search_investigating', 1)
        v.position(3_000_000_000, 12, 22, 5)
        self.event('search_returning', 1)
        self.assertEqual(v.hold_en, [12, 22])


if __name__ == '__main__':
    unittest.main()
