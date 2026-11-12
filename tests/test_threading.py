"""Tests for the thread split and the one-slot buffer, under a deliberately slow stage."""

import threading
import time
import unittest

import fixtures
from fixtures import (requires_cv2, HAVE_CV2, FakeSource, FakeSerial,
                      green_frame, red_frame, blank_frame, wait_until)

if HAVE_CV2:
    import navi_config as cfg
    from navi_vision import LatestOnly, RateCounter, VisionThread, UIThread
    from navi_control import UartThread, STOP_PACKET


@requires_cv2
class TestLatestOnly(unittest.TestCase):
    """The one-slot buffer."""

    def test_holds_one_item(self):
        slot = LatestOnly()
        slot.put("a")
        self.assertEqual(slot.get(timeout=0.1), "a")

    def test_get_empties_the_slot(self):
        slot = LatestOnly()
        slot.put("a")
        slot.get(timeout=0.1)
        self.assertIsNone(slot.get(timeout=0.01))

    def test_peek_does_not_empty_the_slot(self):
        slot = LatestOnly()
        slot.put("a")
        self.assertEqual(slot.peek()[0], "a")
        self.assertEqual(slot.peek()[0], "a")

    def test_newer_item_replaces_the_older_one(self):
        # The property the whole design rests on. A backlog can never form
        # because there is nowhere for one to accumulate.
        slot = LatestOnly()
        for i in range(1000):
            slot.put(i)
        self.assertEqual(slot.get(timeout=0.1), 999)

    def test_overwrites_are_counted_as_drops(self):
        slot = LatestOnly()
        for i in range(10):
            slot.put(i)
        self.assertEqual(slot.dropped, 9)

    def test_consumed_items_are_not_counted_as_drops(self):
        slot = LatestOnly()
        for i in range(10):
            slot.put(i)
            slot.get(timeout=0.1)
        self.assertEqual(slot.dropped, 0)

    def test_memory_does_not_grow(self):
        # Only the latest frame is ever held.
        slot = LatestOnly()
        for i in range(1000):
            slot.put(bytearray(1024))
        self.assertIsNotNone(slot.peek()[0])
        slot.get(timeout=0.1)
        self.assertIsNone(slot.peek()[0])

    def test_get_blocks_until_something_arrives(self):
        slot = LatestOnly()

        def produce():
            time.sleep(0.05)
            slot.put("late")

        threading.Thread(target=produce, daemon=True).start()
        self.assertEqual(slot.get(timeout=1.0), "late")

    def test_get_times_out_rather_than_hanging(self):
        slot = LatestOnly()
        t0 = time.perf_counter()
        self.assertIsNone(slot.get(timeout=0.05))
        self.assertLess(time.perf_counter() - t0, 1.0)

    def test_is_safe_under_concurrent_access(self):
        slot = LatestOnly()
        stop = threading.Event()
        errors = []

        def producer():
            try:
                while not stop.is_set():
                    slot.put(time.perf_counter())
            except Exception as exc:
                errors.append(exc)

        def consumer():
            try:
                while not stop.is_set():
                    slot.get(timeout=0.01)
                    slot.peek()
            except Exception as exc:
                errors.append(exc)

        threads = [threading.Thread(target=producer, daemon=True) for _ in range(2)]
        threads += [threading.Thread(target=consumer, daemon=True) for _ in range(2)]
        for t in threads:
            t.start()
        time.sleep(0.3)
        stop.set()
        for t in threads:
            t.join(timeout=2.0)
        self.assertEqual(errors, [])


@requires_cv2
class TestRateCounter(unittest.TestCase):

    def test_counts_every_tick(self):
        r = RateCounter()
        for _ in range(50):
            r.tick()
        self.assertEqual(r.total, 50)

    def test_reports_zero_before_two_ticks(self):
        r = RateCounter()
        self.assertEqual(r.fps(), 0.0)
        r.tick()
        self.assertEqual(r.fps(), 0.0)

    def test_measures_a_known_rate(self):
        r = RateCounter()
        for _ in range(12):
            r.tick()
            time.sleep(0.01)          # about 100 per second
        self.assertGreater(r.fps(), 30)
        self.assertLess(r.fps(), 300)

    def test_window_is_bounded(self):
        # The window is what keeps this a current rate rather than a lifetime
        # average, and it also keeps memory flat over a long run.
        r = RateCounter(window=10)
        for _ in range(1000):
            r.tick()
        self.assertLessEqual(len(r._times), 10)
        self.assertEqual(r.total, 1000)


@requires_cv2
class TestVisionThread(unittest.TestCase):

    def _start(self, frames, **kw):
        source = FakeSource(frames)
        thread = VisionThread(source, **kw)
        thread.start()
        self.addCleanup(lambda: (thread.stop(), thread.join(timeout=2.0)))
        return source, thread

    def test_produces_detections(self):
        _, vision = self._start([green_frame()])
        self.assertTrue(wait_until(lambda: vision.results.peek()[0] is not None))
        item = vision.results.peek()[0]
        self.assertEqual(item["det"]["command"], "MOVE FORWARD")

    def test_records_per_frame_timings(self):
        _, vision = self._start([green_frame()])
        self.assertTrue(wait_until(lambda: len(vision.frame_ms) > 5))
        self.assertTrue(all(ms > 0 for ms in vision.frame_ms))

    def test_survives_a_source_returning_nothing(self):
        # A live camera drops frames. That must not end the thread, and it must
        # not produce a fabricated detection to paper over the gap.
        source, vision = self._start([green_frame()])
        self.assertTrue(wait_until(lambda: vision.rate.total > 3))
        source.block = True
        before = vision.rate.total
        time.sleep(0.1)
        self.assertTrue(vision.is_alive())
        self.assertLessEqual(vision.rate.total - before, 2)
        source.block = False
        self.assertTrue(wait_until(lambda: vision.rate.total > before + 3))

    def test_consumes_depth_when_provided(self):
        import navi_depth
        depth = navi_depth.zone_map(fixtures.depth_with_obstacle("center", 400.0))
        _, vision = self._start([green_frame()], depth_provider=lambda: depth)
        self.assertTrue(wait_until(lambda: vision.results.peek()[0] is not None))
        det = vision.results.peek()[0]["det"]
        # depth blocked overrides the green target
        self.assertEqual(det["command"], "STOP")

    def test_stops_cleanly(self):
        source = FakeSource([green_frame()])
        vision = VisionThread(source)
        vision.start()
        self.assertTrue(wait_until(lambda: vision.rate.total > 2))
        vision.stop()
        vision.join(timeout=2.0)
        self.assertFalse(vision.is_alive())
        self.assertIsNone(vision.error)


@requires_cv2
class TestUiDoesNotBlockVision(unittest.TestCase):
    """A slow UI must not slow vision."""

    def test_a_slow_ui_does_not_throttle_detection(self):
        # A UI redrawing far slower than the camera delivers is exactly the
        # situation 6.18 warns about. Vision must keep its own pace.
        source = FakeSource([green_frame()])
        vision = VisionThread(source)
        ui = UIThread(vision)
        ui._stop_event.wait = lambda t: time.sleep(min(t, 0.05) if t else 0.05)

        vision.start()
        ui.start()
        try:
            time.sleep(0.5)
            vision_frames = vision.rate.total
            ui_frames = ui.rate.total
        finally:
            ui.stop()
            vision.stop()
            ui.join(timeout=2.0)
            vision.join(timeout=2.0)

        self.assertGreater(vision_frames, 0)
        self.assertGreater(ui_frames, 0)
        # Vision ran many times for each UI redraw. If the two numbers track
        # each other, the threads are coupled and the split has regressed.
        self.assertGreater(vision_frames, ui_frames * 2)

    def test_vision_drops_frames_rather_than_queueing_them(self):
        source = FakeSource([green_frame()])
        vision = VisionThread(source)
        vision.start()
        try:
            self.assertTrue(wait_until(lambda: vision.rate.total > 20))
            # Nothing is consuming, so almost every frame should have been
            # dropped rather than accumulating anywhere.
            self.assertGreater(vision.results.dropped, 10)
        finally:
            vision.stop()
            vision.join(timeout=2.0)

    def test_ui_redraws_even_without_a_new_detection(self):
        # The window has to stay responsive when vision is stalled, otherwise
        # the operator loses the live feed exactly when something is wrong.
        source = FakeSource([green_frame()])
        vision = VisionThread(source)
        ui = UIThread(vision)
        vision.start()
        ui.start()
        try:
            self.assertTrue(wait_until(lambda: ui.rate.total > 2))
            source.block = True
            before = ui.rate.total
            time.sleep(0.2)
            self.assertGreater(ui.rate.total, before)
        finally:
            ui.stop()
            vision.stop()
            ui.join(timeout=2.0)
            vision.join(timeout=2.0)


@requires_cv2
class TestUartIsIndependent(unittest.TestCase):
    """Thread 3 must keep the link alive whatever vision is doing."""

    def test_transmission_continues_while_vision_is_stalled(self):
        # The Pi keeps sending even when the camera stalls.
        source = FakeSource([green_frame()])
        vision = VisionThread(source)
        port = FakeSerial()
        state = {"packet": "<150,150,0>", "t": time.perf_counter()}
        uart = UartThread(port, lambda: (state["packet"], state["t"]))

        vision.start()
        uart.start()
        try:
            self.assertTrue(wait_until(lambda: vision.rate.total > 3))
            source.block = True
            before = len(port.packets())
            time.sleep(0.3)
            after = len(port.packets())
        finally:
            uart.stop()
            vision.stop()
            uart.join(timeout=2.0)
            vision.join(timeout=2.0)

        self.assertGreater(after, before,
                           "UART went quiet when vision stalled")

    def test_stalled_vision_eventually_becomes_a_stop(self):
        # Keeping the link alive is not the same as pretending the last command
        # is still valid. Past the staleness window the packet becomes a stop.
        port = FakeSerial()
        stamp = time.perf_counter()
        uart = UartThread(port, lambda: ("<200,200,0>", stamp))
        uart.start()
        try:
            time.sleep(cfg.COMMAND_STALE_S + 0.25)
        finally:
            uart.stop()
            uart.join(timeout=2.0)
        packets = port.packets()
        self.assertIn("<200,200,0>", packets)     # drove at first
        self.assertEqual(packets[-1], STOP_PACKET)  # then gave up


@requires_cv2
class TestShutdown(unittest.TestCase):

    def test_all_three_threads_stop_without_hanging(self):
        source = FakeSource([green_frame()])
        port = FakeSerial()
        vision = VisionThread(source)
        ui = UIThread(vision)
        uart = UartThread(port, lambda: ("<0,0,0>", time.perf_counter()))

        for t in (vision, ui, uart):
            t.start()
        self.assertTrue(wait_until(lambda: vision.rate.total > 2))

        t0 = time.perf_counter()
        for t in (ui, vision, uart):
            t.stop()
        for t in (ui, vision, uart):
            t.join(timeout=3.0)
        elapsed = time.perf_counter() - t0

        for t in (vision, ui, uart):
            self.assertFalse(t.is_alive(), "%s did not stop" % t.name)
        self.assertLess(elapsed, 3.0)

    def test_threads_report_no_errors(self):
        source = FakeSource([green_frame(), red_frame(), blank_frame()])
        vision = VisionThread(source)
        ui = UIThread(vision)
        vision.start()
        ui.start()
        time.sleep(0.3)
        ui.stop()
        vision.stop()
        ui.join(timeout=2.0)
        vision.join(timeout=2.0)
        self.assertIsNone(vision.error)
        self.assertIsNone(ui.error)


if __name__ == "__main__":
    unittest.main(verbosity=2)
