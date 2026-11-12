"""Tests for the ToF zone logic. Needs numpy only."""

import unittest

import numpy as np

import fixtures
from fixtures import flat_depth, depth_with_obstacle

import navi_config as cfg
from navi_depth import (zone_map, zone_stats, CLEAR, CAUTION, BLOCKED, UNKNOWN,
                        SimDepthSource, DepthReader)


class TestZoneSplitting(unittest.TestCase):

    def test_three_zones_are_produced(self):
        z = zone_map(flat_depth())["zones"]
        self.assertEqual(set(z.keys()), {"left", "center", "right"})

    def test_zones_cover_the_whole_frame(self):
        # No column may be dropped between zones. A gap here would be a blind
        # stripe the chair cannot see obstacles in.
        depth = flat_depth()
        z = zone_map(depth)["zones"]
        total = sum(zone["total_px"] for zone in z.values())
        self.assertEqual(total, depth.size)

    def test_rejects_a_non_2d_map(self):
        with self.assertRaises(ValueError):
            zone_map(np.zeros(100, dtype=np.float32))

    def test_odd_widths_do_not_lose_columns(self):
        for width in (239, 240, 241, 7):
            depth = np.full((10, width), 3000.0, dtype=np.float32)
            z = zone_map(depth)["zones"]
            total = sum(zone["total_px"] for zone in z.values())
            self.assertEqual(total, depth.size, "width %d lost columns" % width)


class TestVerdicts(unittest.TestCase):

    def test_far_is_clear(self):
        self.assertEqual(zone_stats(np.full((10, 10), 3000.0))["verdict"], CLEAR)

    def test_middle_distance_is_caution(self):
        mid = (cfg.ZONE_BLOCKED_MM + cfg.ZONE_CAUTION_MM) / 2.0
        self.assertEqual(zone_stats(np.full((10, 10), mid))["verdict"], CAUTION)

    def test_close_is_blocked(self):
        self.assertEqual(zone_stats(np.full((10, 10), 400.0))["verdict"], BLOCKED)

    def test_threshold_boundaries(self):
        # Exactly on a threshold the less severe verdict applies.
        at_blocked = np.full((10, 10), float(cfg.ZONE_BLOCKED_MM))
        self.assertEqual(zone_stats(at_blocked)["verdict"], CAUTION)

        just_under = np.full((10, 10), float(cfg.ZONE_BLOCKED_MM - 1))
        self.assertEqual(zone_stats(just_under)["verdict"], BLOCKED)

        at_caution = np.full((10, 10), float(cfg.ZONE_CAUTION_MM))
        self.assertEqual(zone_stats(at_caution)["verdict"], CLEAR)


class TestInvalidData(unittest.TestCase):
    """A ToF that cannot see is not a ToF reporting a clear path."""

    def test_all_zero_zone_is_unknown(self):
        # Zero is a dropout, not an object at the lens.
        s = zone_stats(np.zeros((20, 20), dtype=np.float32))
        self.assertEqual(s["verdict"], UNKNOWN)
        self.assertIsNone(s["distance_mm"])

    def test_out_of_range_pixels_are_excluded(self):
        beyond = np.full((20, 20), float(cfg.DEPTH_MAX_MM + 5000))
        self.assertEqual(zone_stats(beyond)["verdict"], UNKNOWN)

    def test_unknown_center_blocks_the_path(self):
        # Blind ahead counts as blocked.
        depth = flat_depth(0.0)
        result = zone_map(depth)
        self.assertEqual(result["zones"]["center"]["verdict"], UNKNOWN)
        self.assertTrue(result["path_blocked"])
        self.assertIn("NO VALID DEPTH", result["reason"])

    def test_empty_zone_is_unknown_not_a_crash(self):
        s = zone_stats(np.zeros((0, 0), dtype=np.float32))
        self.assertEqual(s["verdict"], UNKNOWN)

    def test_mostly_invalid_zone_is_unknown(self):
        # A handful of valid pixels in a sea of dropouts is not enough to
        # believe. The valid fraction floor is what enforces that.
        depth = np.zeros((100, 100), dtype=np.float32)
        depth[:2, :] = 3000.0                   # 2% valid, floor is 10%
        s = zone_stats(depth)
        self.assertEqual(s["verdict"], UNKNOWN)

    def test_enough_valid_pixels_is_trusted(self):
        depth = np.zeros((100, 100), dtype=np.float32)
        depth[:20, :] = 3000.0                  # 20% valid, over the floor
        s = zone_stats(depth)
        self.assertEqual(s["verdict"], CLEAR)
        self.assertEqual(s["distance_mm"], 3000)


class TestMedianRobustness(unittest.TestCase):
    """Why the median, and not the mean or the minimum."""

    def test_a_few_bad_pixels_do_not_move_the_reading(self):
        depth = np.full((100, 100), 3000.0, dtype=np.float32)
        depth[0, :5] = 250.0                    # five specular reflections
        s = zone_stats(depth)
        self.assertEqual(s["verdict"], CLEAR)
        self.assertGreater(s["distance_mm"], 2900)

    def test_minimum_would_have_failed_that_case(self):
        # Percentile, not minimum: five bad pixels must not stop the chair.
        depth = np.full((100, 100), 3000.0, dtype=np.float32)
        depth[0, :5] = 250.0
        valid = depth[(depth >= cfg.DEPTH_MIN_MM) & (depth <= cfg.DEPTH_MAX_MM)]
        self.assertLess(float(valid.min()), cfg.ZONE_BLOCKED_MM)
        self.assertEqual(zone_stats(depth)["verdict"], CLEAR)

    def test_a_real_obstacle_still_registers(self):
        # The robustness above must not have made the sensor deaf. Half the
        # zone at close range is an obstacle, not noise.
        depth = np.full((100, 100), 3000.0, dtype=np.float32)
        depth[:60, :] = 400.0
        self.assertEqual(zone_stats(depth)["verdict"], BLOCKED)


class TestCaptureRegressions(unittest.TestCase):
    """Two misses found with tof_capture.py."""

    def test_small_obstacle_is_not_outvoted_by_the_background(self):
        # A hand at 30 cm filled about a quarter of the center zone. The median
        # reported the ceiling behind it and called the zone CLEAR.
        depth = np.full((100, 100), 1900.0, dtype=np.float32)
        depth[:25, :] = 250.0
        self.assertEqual(zone_stats(depth)["verdict"], BLOCKED)

    def test_too_close_readings_count_as_an_obstacle(self):
        # Very close readings are kept: a hand at 15 cm must not read CLEAR.
        depth = np.full((100, 100), 3000.0, dtype=np.float32)
        depth[:70, :] = 15.0
        self.assertEqual(zone_stats(depth)["verdict"], BLOCKED)

    def test_about_five_percent_is_the_trigger(self):
        depth = np.full((100, 100), 3000.0, dtype=np.float32)
        depth[:3, :] = 300.0                    # 3%: treated as noise
        self.assertEqual(zone_stats(depth)["verdict"], CLEAR)
        depth[:8, :] = 300.0                    # 8%: a real object
        self.assertEqual(zone_stats(depth)["verdict"], BLOCKED)


class TestObstacleLocalisation(unittest.TestCase):
    """Which side an obstacle is on."""

    def test_center_obstacle_blocks_the_path(self):
        r = zone_map(depth_with_obstacle("center", 400.0))
        self.assertTrue(r["path_blocked"])
        self.assertEqual(r["zones"]["center"]["verdict"], BLOCKED)
        self.assertEqual(r["zones"]["left"]["verdict"], CLEAR)
        self.assertEqual(r["zones"]["right"]["verdict"], CLEAR)

    def test_left_obstacle_leaves_the_path_open(self):
        r = zone_map(depth_with_obstacle("left", 400.0))
        self.assertFalse(r["path_blocked"])
        self.assertEqual(r["zones"]["left"]["verdict"], BLOCKED)

    def test_right_obstacle_leaves_the_path_open(self):
        r = zone_map(depth_with_obstacle("right", 400.0))
        self.assertFalse(r["path_blocked"])
        self.assertEqual(r["zones"]["right"]["verdict"], BLOCKED)

    def test_suggested_turn_points_at_the_open_side(self):
        depth = depth_with_obstacle("center", 400.0)
        depth[:, : depth.shape[1] // 3] = 900.0      # left is tighter
        r = zone_map(depth)
        self.assertTrue(r["path_blocked"])
        self.assertEqual(r["suggested_turn"], "right")

    def test_suggested_turn_flips_with_the_geometry(self):
        depth = depth_with_obstacle("center", 400.0)
        depth[:, 2 * depth.shape[1] // 3:] = 900.0   # right is tighter
        self.assertEqual(zone_map(depth)["suggested_turn"], "left")

    def test_no_suggestion_when_the_path_is_clear(self):
        self.assertIsNone(zone_map(flat_depth(3000.0))["suggested_turn"])

    def test_no_suggestion_when_both_sides_are_blind(self):
        depth = np.zeros_like(flat_depth())
        self.assertIsNone(zone_map(depth)["suggested_turn"])


class TestSimSource(unittest.TestCase):

    def test_produces_correctly_shaped_maps(self):
        src = SimDepthSource()
        depth = src.read()
        self.assertEqual(depth.shape, (cfg.DEPTH_HEIGHT, cfg.DEPTH_WIDTH))

    def test_sweep_eventually_blocks_the_center(self):
        # The simulated obstacle has to actually exercise the blocked path, or
        # the bench demo proves nothing.
        src = SimDepthSource()
        saw_blocked = False
        for _ in range(200):
            if zone_map(src.read())["path_blocked"]:
                saw_blocked = True
                break
        self.assertTrue(saw_blocked)

    def test_sweep_also_produces_clear_frames(self):
        src = SimDepthSource()
        self.assertTrue(any(not zone_map(src.read())["path_blocked"]
                            for _ in range(200)))


class TestDepthReader(unittest.TestCase):
    """The background poller the vision thread reads from."""

    def test_publishes_the_latest_summary(self):
        src = fixtures.FakeDepthSource([depth_with_obstacle("center", 400.0)])
        reader = DepthReader(source=src).start()
        try:
            self.assertTrue(fixtures.wait_until(
                lambda: reader.latest() is not None))
            self.assertTrue(reader.latest()["path_blocked"])
        finally:
            reader.stop()
        self.assertTrue(src.closed)

    def test_latest_is_none_before_the_first_frame(self):
        src = fixtures.FakeDepthSource([])
        reader = DepthReader(source=src)
        self.assertIsNone(reader.latest())
        reader.stop()

    def test_reader_never_blocks_its_caller(self):
        # The vision thread calls latest() once per frame at 30 fps or better,
        # so it must return immediately whether or not new depth has arrived.
        import time
        src = fixtures.FakeDepthSource([flat_depth()])
        reader = DepthReader(source=src).start()
        try:
            fixtures.wait_until(lambda: reader.latest() is not None)
            t0 = time.perf_counter()
            for _ in range(1000):
                reader.latest()
            elapsed = time.perf_counter() - t0
            self.assertLess(elapsed, 0.5)
        finally:
            reader.stop()


if __name__ == "__main__":
    unittest.main(verbosity=2)
