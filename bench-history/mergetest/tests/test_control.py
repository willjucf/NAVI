"""Virtual tests for the state machine and the UART transmit thread.

The FSM tests are the software version of the emergency stop integration case
in report 8.4.1. That case measures a real button against an oscilloscope and
needs hardware. What it is really checking, that a stop wins from any state and
that leaving it requires a deliberate reset, is logic, and logic can be checked
exhaustively here first.

The UART tests assert on the exact bytes handed to the port, using a fake serial
object, so what goes on the wire is verified rather than assumed.
"""

import time
import unittest

import fixtures
from fixtures import (requires_cv2, HAVE_CV2, FakeSerial, FakeSource,
                      green_frame, red_frame)

if HAVE_CV2:
    import navi_config as cfg
    from navi_vision import detect
    from navi_control import (FSM, MANUAL, AUTO, ESTOP, UartThread,
                              format_packet, STOP_PACKET, NullPort)


@requires_cv2
class TestFsmTransitions(unittest.TestCase):
    """Report 6.15. Three states, three events."""

    def test_starts_where_it_was_told_to(self):
        self.assertEqual(FSM(start=AUTO).state, AUTO)
        self.assertEqual(FSM(start=MANUAL).state, MANUAL)

    def test_toggle_switches_between_manual_and_auto(self):
        fsm = FSM(start=MANUAL)
        self.assertEqual(fsm.event("toggle_mode"), AUTO)
        self.assertEqual(fsm.event("toggle_mode"), MANUAL)

    def test_unknown_events_are_ignored(self):
        fsm = FSM(start=AUTO)
        fsm.event("launch")
        self.assertEqual(fsm.state, AUTO)

    def test_there_are_exactly_three_states(self):
        # Report 6.15 defines three and argues explicitly that initialization
        # is a pre loop routine rather than a fourth state. If a fourth ever
        # appears, that section of the report is now wrong.
        fsm = FSM(start=AUTO)
        seen = set()
        for ev in ("toggle_mode", "estop", "reset", "toggle_mode",
                   "estop", "reset", "toggle_mode"):
            fsm.event(ev)
            seen.add(fsm.state)
        self.assertEqual(seen, {MANUAL, AUTO, ESTOP})


@requires_cv2
class TestEstopSafetyProperties(unittest.TestCase):
    """The two properties report 6.15 and 6.21 name as verified."""

    def test_estop_latches_from_every_state(self):
        for start in (MANUAL, AUTO, ESTOP):
            fsm = FSM(start=start)
            self.assertEqual(fsm.event("estop"), ESTOP,
                             "estop was refused from %s" % start)

    def test_toggle_cannot_escape_estop(self):
        # The FMEA in report 6.21 lists this as a tested failure mode: a stray
        # toggle must not be able to resume driving after a safety stop.
        fsm = FSM(start=AUTO)
        fsm.event("estop")
        for _ in range(10):
            fsm.event("toggle_mode")
            self.assertEqual(fsm.state, ESTOP)

    def test_reset_always_lands_in_manual(self):
        # Never straight back into AUTO. Resuming autonomous driving has to
        # take a second deliberate action after a stop.
        fsm = FSM(start=AUTO)
        fsm.event("estop")
        self.assertEqual(fsm.event("reset"), MANUAL)

    def test_reset_does_nothing_outside_estop(self):
        for start in (MANUAL, AUTO):
            fsm = FSM(start=start)
            fsm.event("reset")
            self.assertEqual(fsm.state, start)

    def test_returning_to_auto_needs_a_separate_toggle(self):
        fsm = FSM(start=AUTO)
        fsm.event("estop")
        fsm.event("reset")
        self.assertEqual(fsm.state, MANUAL)
        self.assertEqual(fsm.event("toggle_mode"), AUTO)

    def test_the_scripted_sequence_from_the_report(self):
        # The exact run cited in report 6.15 and 8.4.1.
        fsm = FSM(start=AUTO)
        fsm.event("estop")
        fsm.event("reset")
        self.assertEqual(fsm.state, MANUAL)
        self.assertEqual(fsm.history, [AUTO, ESTOP, MANUAL])


@requires_cv2
class TestFsmResolve(unittest.TestCase):
    """What each state actually puts on the wire."""

    def test_estop_forces_the_stop_packet(self):
        # Regardless of what vision is reporting in the same instant.
        fsm = FSM(start=AUTO)
        fsm.event("estop")
        moving = detect(green_frame())
        command, packet, flag = fsm.resolve(moving)
        self.assertEqual(packet, "<0,0,1>")
        self.assertEqual(flag, 1)

    def test_manual_is_idle_not_moving(self):
        # Report 6.8. Until the joystick is wired, MANUAL emits an idle packet,
        # so a system sitting in MANUAL with nothing connected does not move
        # rather than acting on undefined input.
        fsm = FSM(start=MANUAL)
        command, packet, flag = fsm.resolve(detect(green_frame()))
        self.assertEqual(packet, "<0,0,0>")

    def test_auto_follows_the_vision_result(self):
        fsm = FSM(start=AUTO)
        det = detect(green_frame())
        command, packet, flag = fsm.resolve(det)
        self.assertEqual(packet, det["packet"])

    def test_auto_passes_a_red_hazard_through(self):
        fsm = FSM(start=AUTO)
        command, packet, flag = fsm.resolve(detect(red_frame()))
        self.assertEqual(packet, "<0,0,1>")

    def test_auto_with_no_frame_yet_stops(self):
        # Startup, or a camera that has not delivered. Do not invent a command.
        fsm = FSM(start=AUTO)
        command, packet, flag = fsm.resolve(None)
        self.assertEqual(packet, "<0,0,1>")

    def test_estop_overrides_even_a_missing_frame(self):
        fsm = FSM(start=AUTO)
        fsm.event("estop")
        self.assertEqual(fsm.resolve(None)[1], "<0,0,1>")


@requires_cv2
class TestPacketFormatting(unittest.TestCase):
    """Report 6.13, the exact wire format."""

    def test_basic_shape(self):
        self.assertEqual(format_packet(150, 150, 0), "<150,150,0>")
        self.assertEqual(format_packet(0, 0, 1), "<0,0,1>")

    def test_negative_values(self):
        self.assertEqual(format_packet(-255, -100, 0), "<-255,-100,0>")

    def test_clamps_out_of_range(self):
        # This function is the last thing between a number and the motor
        # controller, so it clamps again even though steering_packet already
        # did. Defence in depth on the one value that moves the chair.
        self.assertEqual(format_packet(9000, -9000, 0), "<255,-255,0>")

    def test_flag_is_coerced_to_a_boolean(self):
        self.assertEqual(format_packet(0, 0, True), "<0,0,1>")
        self.assertEqual(format_packet(0, 0, 7), "<0,0,1>")
        self.assertEqual(format_packet(0, 0, 0), "<0,0,0>")

    def test_floats_do_not_leak_into_the_packet(self):
        # A float would serialise as 150.0 and the ESP32 parser would reject
        # the frame, so the chair would coast until the watchdog fired.
        self.assertEqual(format_packet(150.7, -20.2, 0), "<150,-20,0>")

    def test_stop_packet_constant_matches(self):
        self.assertEqual(STOP_PACKET, format_packet(0, 0, 1))


@requires_cv2
class TestUartThread(unittest.TestCase):
    """Thread 3. What actually reaches the port."""

    def _run(self, source, duration=0.3, port=None):
        port = port or FakeSerial()
        thread = UartThread(port, source)
        thread.start()
        time.sleep(duration)
        thread.stop()
        thread.join(timeout=2.0)
        return port, thread

    def test_transmits_the_current_command(self):
        port, _ = self._run(lambda: ("<150,150,0>", time.perf_counter()))
        self.assertTrue(port.packets())
        self.assertTrue(all(p == "<150,150,0>" for p in port.packets()))

    def test_every_packet_is_well_formed(self):
        port, _ = self._run(lambda: ("<-42,200,1>", time.perf_counter()))
        for p in port.packets():
            self.assertRegex(p, r"^<-?\d+,-?\d+,[01]>$")

    def test_transmits_at_roughly_the_configured_rate(self):
        # The whole reason this thread exists is to keep the ESP32 loss of
        # signal watchdog fed, so the rate is the behaviour under test.
        duration = 0.5
        port, _ = self._run(lambda: ("<0,0,0>", time.perf_counter()), duration)
        expected = duration / cfg.UART_TX_INTERVAL_S
        self.assertGreater(len(port.packets()), expected * 0.5)

    def test_interval_is_well_inside_the_watchdog_window(self):
        # If someone raises the interval past the 500 ms window, the chair will
        # halt every cycle. Pin the relationship rather than the number.
        self.assertLess(cfg.UART_TX_INTERVAL_S * 1000, cfg.UART_WATCHDOG_MS / 2)

    def test_no_command_yet_sends_a_stop(self):
        port, _ = self._run(lambda: (None, None))
        self.assertTrue(port.packets())
        self.assertTrue(all(p == STOP_PACKET for p in port.packets()))

    def test_stale_command_is_downgraded_to_a_stop(self):
        # Vision has stalled. Rather than keep driving on information it no
        # longer trusts, the thread sends a stop on its own.
        old = time.perf_counter() - (cfg.COMMAND_STALE_S * 4)
        port, thread = self._run(lambda: ("<200,200,0>", old))
        self.assertTrue(all(p == STOP_PACKET for p in port.packets()))
        self.assertGreater(thread.stale_downgrades, 0)

    def test_fresh_command_is_not_downgraded(self):
        port, thread = self._run(lambda: ("<200,200,0>", time.perf_counter()))
        self.assertEqual(thread.stale_downgrades, 0)
        self.assertTrue(all(p == "<200,200,0>" for p in port.packets()))

    def test_a_write_failure_does_not_kill_the_thread(self):
        # A dead link is the ESP32 watchdog problem, not a reason for the Pi to
        # stop trying. If this thread died on the first error, the link could
        # never recover on its own.
        port = FakeSerial(fail_after=2)
        thread = UartThread(port, lambda: ("<0,0,0>", time.perf_counter()))
        thread.start()
        time.sleep(0.3)
        self.assertTrue(thread.is_alive())
        thread.stop()
        thread.join(timeout=2.0)
        self.assertGreater(thread.errors, 0)

    def test_stops_cleanly(self):
        port = FakeSerial()
        thread = UartThread(port, lambda: ("<0,0,0>", time.perf_counter()))
        thread.start()
        time.sleep(0.1)
        thread.stop()
        thread.join(timeout=2.0)
        self.assertFalse(thread.is_alive())

    def test_null_port_accepts_writes(self):
        port = NullPort()
        port.write(b"<0,0,1>")
        self.assertTrue(port.buffer)
        port.close()


@requires_cv2
class TestEndToEnd(unittest.TestCase):
    """Vision to FSM to wire, with fake hardware at both ends."""

    def _pipeline(self, frames, events=(), duration=0.35):
        from navi_vision import VisionThread
        source = FakeSource(frames)
        vision = VisionThread(source)
        port = FakeSerial()

        state = {"packet": None, "t": None}
        fsm = FSM(start=AUTO)
        for ev in events:
            fsm.event(ev)

        uart = UartThread(port, lambda: (state["packet"], state["t"]))
        vision.start()
        uart.start()
        deadline = time.perf_counter() + duration
        try:
            while time.perf_counter() < deadline:
                item = vision.results.peek()[0]
                det = item["det"] if item else None
                state["packet"] = fsm.resolve(det)[1]
                state["t"] = time.perf_counter()
                time.sleep(0.002)
        finally:
            vision.stop()
            uart.stop()
            vision.join(timeout=2.0)
            uart.join(timeout=2.0)
        return port.packets()

    def test_green_target_produces_a_drive_command(self):
        packets = self._pipeline([green_frame()])
        self.assertTrue(packets)
        self.assertIn("<150,150,0>", packets)

    def test_red_hazard_produces_the_stop_packet(self):
        # This is the Step 3 acceptance check: simulate an obstacle and confirm
        # <0,0,1> reaches the wire.
        packets = self._pipeline([red_frame()])
        self.assertTrue(packets)
        self.assertTrue(all(p == "<0,0,1>" for p in packets))

    def test_estop_overrides_a_live_green_target(self):
        packets = self._pipeline([green_frame()], events=("estop",))
        self.assertTrue(packets)
        self.assertTrue(all(p == "<0,0,1>" for p in packets))

    def test_manual_mode_stays_idle_on_a_green_target(self):
        packets = self._pipeline([green_frame()], events=("toggle_mode",))
        self.assertTrue(packets)
        # The idle command has to appear, and nothing that moves the chair may.
        # Asserting on motion rather than on an exact string is deliberate: the
        # very first packet is a stop, sent before any command exists at all,
        # and that startup behaviour is correct rather than something to
        # tolerate. What matters is that MANUAL never emits a drive value.
        self.assertIn("<0,0,0>", packets)
        for packet in packets:
            left, right, _ = packet.strip("<>").split(",")
            self.assertEqual((int(left), int(right)), (0, 0),
                             "MANUAL emitted motion: %s" % packet)


if __name__ == "__main__":
    unittest.main(verbosity=2)
