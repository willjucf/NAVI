"""Tests for the detection pipeline on synthetic frames."""

import unittest

from fixtures import (requires_cv2, HAVE_CV2, blank_frame, green_frame,
                      red_frame, red_and_green_frame, glare_frame,
                      speckle_frame, flat_depth, depth_with_obstacle)

if HAVE_CV2:
    import navi_config as cfg
    import navi_depth
    from navi_vision import detect, normalize, annotate


@requires_cv2
class TestColourDecisions(unittest.TestCase):
    """The four cases the command logic has to get right."""

    def test_green_target_moves_forward(self):
        d = detect(green_frame())
        self.assertEqual(d["command"], "MOVE FORWARD")
        self.assertEqual(d["flag"], 0)

    def test_centered_green_drives_straight(self):
        d = detect(green_frame(cx_frac=0.5))
        self.assertEqual(d["packet"], "<150,150,0>")

    def test_red_hazard_stops(self):
        d = detect(red_frame())
        self.assertEqual(d["command"], "STOP")
        self.assertEqual(d["flag"], 1)
        self.assertEqual(d["packet"], "<0,0,1>")

    def test_empty_scene_stops(self):
        # Nothing found means stop, never coast.
        d = detect(blank_frame())
        self.assertEqual(d["command"], "STOP")
        self.assertEqual(d["packet"], "<0,0,0>")

    def test_no_target_flag_is_not_a_hazard(self):
        # Nothing seen is a stop, but it is not a hazard, so the safety flag
        # stays 0. Only a real hazard raises it.
        self.assertEqual(detect(blank_frame())["flag"], 0)
        self.assertEqual(detect(red_frame())["flag"], 1)


@requires_cv2
class TestRedBeatsGreen(unittest.TestCase):
    """The single most important rule in the pipeline."""

    def test_red_wins_when_both_are_present(self):
        d = detect(red_and_green_frame())
        self.assertEqual(d["command"], "STOP")
        self.assertEqual(d["packet"], "<0,0,1>")

    def test_the_green_target_really_was_detected(self):
        # Green alone is detected in this frame, so the test above proves priority.
        d = detect(red_and_green_frame())
        self.assertIsNotNone(d["green_c"])
        self.assertGreater(d["green_area"], 0)
        self.assertIsNotNone(d["red_c"])
        # and it stopped anyway
        self.assertEqual(d["command"], "STOP")

    def test_red_wins_even_when_green_is_larger(self):
        # Priority is by colour, never by size. A small hazard beats a big
        # clear path marker.
        from fixtures import add_blob, BGR_GREEN, BGR_RED
        frame = blank_frame()
        add_blob(frame, BGR_GREEN, cx_frac=0.3, size_frac=0.45)
        add_blob(frame, BGR_RED, cx_frac=0.8, size_frac=0.12)
        d = detect(frame)
        self.assertGreater(d["green_area"], d["red_area"])
        self.assertEqual(d["command"], "STOP")


@requires_cv2
class TestCalibrationRegressions(unittest.TestCase):
    """Glare read as green, and dim red missed."""

    def test_glare_is_not_mistaken_for_green(self):
        # Glare used to read as green.
        d = detect(glare_frame())
        self.assertNotEqual(
            d["command"], "MOVE FORWARD",
            "glare was accepted as a green target, the saturation floor has "
            "regressed below the measured value")

    def test_dim_red_is_still_caught(self):
        # Dim red used to be missed.
        from fixtures import add_blob
        dim_red = (25, 25, 110)          # dark but clearly red
        frame = blank_frame(color=(30, 30, 30))
        add_blob(frame, dim_red, size_frac=0.3)
        d = detect(frame)
        self.assertEqual(d["command"], "STOP")
        self.assertEqual(d["flag"], 1)

    def test_small_red_speck_does_not_override_green(self):
        # Skin at the knuckles used to pass as red.
        from fixtures import add_blob, BGR_RED
        frame = green_frame(cx_frac=0.3)
        add_blob(frame, BGR_RED, cx_frac=0.85, size_frac=0.07)   # about 0.9% of frame
        self.assertEqual(detect(frame)["command"], "MOVE FORWARD")

    def test_a_real_red_marker_still_wins_over_green(self):
        from fixtures import add_blob, BGR_RED
        frame = green_frame(cx_frac=0.3)
        add_blob(frame, BGR_RED, cx_frac=0.8, size_frac=0.15)    # about 4% of frame
        d = detect(frame)
        self.assertEqual(d["command"], "STOP")
        self.assertEqual(d["flag"], 1)

    def test_thresholds_match_the_measured_values(self):
        self.assertEqual(list(cfg.LOWER_GREEN), [35, 50, 40])
        self.assertEqual(list(cfg.UPPER_GREEN), [90, 255, 255])
        self.assertEqual(list(cfg.LOWER_RED_1), [0, 140, 70])
        self.assertEqual(list(cfg.UPPER_RED_1), [7, 255, 255])
        self.assertEqual(list(cfg.LOWER_RED_2), [160, 140, 70])


@requires_cv2
class TestNoiseRejection(unittest.TestCase):

    def test_speckle_is_not_a_target(self):
        # Scattered pixels are not an object.
        d = detect(speckle_frame())
        self.assertEqual(d["command"], "STOP")
        self.assertIsNone(d["green_c"])

    def test_blob_below_minimum_area_is_ignored(self):
        # Just under the 0.4% threshold.
        d = detect(green_frame(size_frac=0.02))
        self.assertIsNone(d["green_c"])
        self.assertEqual(d["command"], "STOP")

    def test_blob_above_minimum_area_is_accepted(self):
        d = detect(green_frame(size_frac=0.25))
        self.assertIsNotNone(d["green_c"])


@requires_cv2
class TestSteering(unittest.TestCase):
    """Differential mixing."""

    def test_target_on_the_right_turns_right(self):
        d = detect(green_frame(cx_frac=0.85))
        self.assertGreater(d["left"], d["right"])
        self.assertIn("RIGHT", d["status"])

    def test_target_on_the_left_turns_left(self):
        d = detect(green_frame(cx_frac=0.15))
        self.assertGreater(d["right"], d["left"])
        self.assertIn("LEFT", d["status"])

    def test_steering_is_symmetric(self):
        # Mirror-image targets give mirror-image commands.
        left = detect(green_frame(cx_frac=0.2))
        right = detect(green_frame(cx_frac=0.8))
        self.assertAlmostEqual(left["left"], right["right"], delta=2)
        self.assertAlmostEqual(left["right"], right["left"], delta=2)

    def test_pwm_never_leaves_the_documented_range(self):
        # Base speed plus turn can exceed 255 before clipping.
        for frac in (0.0, 0.05, 0.5, 0.95, 1.0):
            d = detect(green_frame(cx_frac=frac))
            self.assertGreaterEqual(d["left"], -cfg.PWM_MAX)
            self.assertLessEqual(d["left"], cfg.PWM_MAX)
            self.assertGreaterEqual(d["right"], -cfg.PWM_MAX)
            self.assertLessEqual(d["right"], cfg.PWM_MAX)


@requires_cv2
class TestPacketFormat(unittest.TestCase):
    """Every packet from detect() matches the wire format."""

    def test_packet_shape(self):
        import re
        pattern = re.compile(r"^<-?\d+,-?\d+,[01]>$")
        frames = [blank_frame(), green_frame(), red_frame(),
                  red_and_green_frame(), green_frame(cx_frac=0.05),
                  green_frame(cx_frac=0.95), glare_frame(), speckle_frame()]
        for i, frame in enumerate(frames):
            packet = detect(frame)["packet"]
            self.assertRegex(packet, pattern, "frame %d produced %r" % (i, packet))

    def test_packet_agrees_with_its_own_fields(self):
        d = detect(green_frame(cx_frac=0.7))
        self.assertEqual(d["packet"],
                         "<%d,%d,%d>" % (d["left"], d["right"], d["flag"]))


@requires_cv2
class TestNormalize(unittest.TestCase):

    def test_wide_frames_are_downscaled(self):
        import numpy as np
        big = np.zeros((1080, 1920, 3), dtype=np.uint8)
        self.assertEqual(normalize(big).shape[1], cfg.PROC_WIDTH)

    def test_aspect_ratio_is_preserved(self):
        import numpy as np
        big = np.zeros((1080, 1920, 3), dtype=np.uint8)
        out = normalize(big)
        self.assertAlmostEqual(out.shape[1] / out.shape[0], 1920 / 1080,
                               places=2)

    def test_small_frames_are_left_alone(self):
        small = blank_frame(320, 240)
        self.assertEqual(normalize(small).shape, small.shape)

    def test_decision_survives_rescaling(self):
        # Same scene at a different resolution gives the same command.
        import cv2
        frame = green_frame(cx_frac=0.5, size_frac=0.3)
        big = cv2.resize(frame, (1920, 1440), interpolation=cv2.INTER_NEAREST)
        self.assertEqual(detect(normalize(frame))["command"],
                         detect(normalize(big))["command"])


@requires_cv2
class TestDepthFusion(unittest.TestCase):
    """Depth is a stop trigger and never a reason to move."""

    def test_clear_depth_allows_a_green_target(self):
        depth = navi_depth.zone_map(flat_depth(3000.0))
        d = detect(green_frame(), depth=depth)
        self.assertEqual(d["command"], "MOVE FORWARD")

    def test_blocked_center_overrides_a_green_target(self):
        depth = navi_depth.zone_map(depth_with_obstacle("center", 400.0))
        d = detect(green_frame(), depth=depth)
        self.assertEqual(d["command"], "STOP")
        self.assertEqual(d["flag"], 1)

    def test_blocked_side_does_not_stop_the_chair(self):
        # An obstacle beside the path is not an obstacle in it. Stopping for
        # one would make the chair unusable in any corridor.
        depth = navi_depth.zone_map(depth_with_obstacle("left", 400.0))
        d = detect(green_frame(), depth=depth)
        self.assertEqual(d["command"], "MOVE FORWARD")

    def test_red_still_beats_clear_depth(self):
        # Adding depth must not have weakened the colour priority rule.
        depth = navi_depth.zone_map(flat_depth(3000.0))
        d = detect(red_frame(), depth=depth)
        self.assertEqual(d["command"], "STOP")

    def test_depth_absent_behaves_exactly_as_before(self):
        # The fusion was designed as an additive field so existing behaviour is
        # untouched when no ToF is connected.
        self.assertEqual(detect(green_frame(), depth=None)["packet"],
                         detect(green_frame())["packet"])


@requires_cv2
class TestAnnotate(unittest.TestCase):

    def test_annotate_does_not_modify_the_source_frame(self):
        # annotate() must copy; the UI and vision threads share frames.
        frame = green_frame()
        before = frame.copy()
        annotate(frame, detect(frame))
        self.assertTrue((frame == before).all())

    def test_annotate_returns_a_drawable_image(self):
        frame = green_frame()
        out = annotate(frame, detect(frame), hud="test")
        self.assertEqual(out.shape, frame.shape)
        self.assertFalse((out == frame).all())   # something was actually drawn

    def test_annotate_handles_every_decision_type(self):
        for frame in (blank_frame(), green_frame(), red_frame(),
                      red_and_green_frame()):
            annotate(frame, detect(frame))       # must not raise


if __name__ == "__main__":
    unittest.main(verbosity=2)
