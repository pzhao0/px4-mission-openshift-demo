"""Per-flight, display-only survey evidence, in local east/north metres."""
from collections import deque
from math import dist, isfinite


class SurveyView:
    def __init__(self):
        self.poses = deque(maxlen=256)
        self.previews = deque(maxlen=40)
        self.lanes = []
        self.run = None
        self.phase = None
        self.clear_run()

    def clear_run(self):
        self.trail = []
        self.sighting_stamp = 0
        self.hold_frame_stamp = 0
        self.hold_image = None
        self.first_image = None
        self.first_en = None
        self.hold_en = None
        self.last_path_stamp = 0

    def status(self, mission):
        survey = mission.get('survey', {})
        run = survey.get('run_ns', 0)
        if run != self.run:
            self.clear_run()
            self.run = run
        self.lanes = survey.get('lanes_en', [])
        self.phase = mission.get('phase')
        if self.phase in ('search_spotted', 'search_searching'):
            self.hold_en = None
        stamp = survey.get('sighting_ns', 0)
        if self.sighting_stamp != stamp:
            self.first_image = self.first_en = None
        self.sighting_stamp = stamp
        if self.sighting_stamp and self.first_image is None:
            for stamp, data in reversed(self.previews):
                if stamp == self.sighting_stamp:
                    self.first_image = data
                    break
        hold_stamp = survey.get('hold_frame_ns', 0)
        if hold_stamp != self.hold_frame_stamp:
            self.hold_en = None
            self.hold_frame_stamp = hold_stamp
            self.hold_image = next((data for stamp, data in reversed(self.previews)
                                    if hold_stamp and stamp == hold_stamp), None)
        self.locate_sighting()

    def position(self, stamp, east, north, height):
        if not stamp or not all(map(isfinite, (east, north, height))):
            return
        if self.poses and stamp < self.poses[-1][0]:
            self.poses.clear()
        self.poses.append((stamp, (east, north, height)))
        self.locate_sighting()
        if not self.run or self.phase not in ('demo_takeoff', 'search_searching', 'search_spotted',
                                               'search_investigating', 'search_returning', 'demo_landing'):
            return
        point = [round(east, 2), round(north, 2)]
        if self.phase == 'search_investigating' and self.hold_en is None:
            self.hold_en = point
        if self.trail and stamp - self.last_path_stamp > 500_000_000 and self.trail[-1] is not None:
            self.trail.append(None)
        if not self.trail or self.trail[-1] is None or dist(point, self.trail[-1]) >= .20:
            self.trail.append(point)
        self.last_path_stamp = stamp

    def at_capture(self, stamp):
        # PX4 and camera timestamps have different epochs. Pose receipt is tagged
        # with Gazebo /clock; interpolate only when fresh samples bracket capture.
        after = None
        for before in reversed(self.poses):
            if before[0] == stamp:
                return before[1]
            if before[0] < stamp:
                if after is None or after[0] - before[0] > 200_000_000:
                    return None
                fraction = (stamp - before[0]) / (after[0] - before[0])
                return tuple(a + fraction * (b - a) for a, b in zip(before[1], after[1]))
            after = before
        return None

    def locate_sighting(self):
        if self.sighting_stamp and self.first_en is None:
            pose = self.at_capture(self.sighting_stamp)
            if pose is not None:
                self.first_en = [round(v, 2) for v in pose[:2]]

    def preview(self, stamp, data):
        self.previews.append((stamp, data))
        if stamp and stamp == self.hold_frame_stamp:
            self.hold_image = data
        if stamp == self.sighting_stamp:
            self.first_image = data

    def snapshot(self):
        return dict(run_ns=self.run, lanes=self.lanes, trail=self.trail.copy(),
                    hold_photo_ns=self.hold_frame_stamp if self.hold_image is not None else 0,
                    first_en=self.first_en, hold_en=self.hold_en,
                    sighting_ns=self.sighting_stamp, has_image=self.first_image is not None)
