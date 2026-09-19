'''Tests for the frame profiler.

Run from the repository root:

    python -m unittest discover -s tests -t .
'''

import contextlib
import gc
import io
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mgllib.profiler import FrameProfiler, GCTracker, active_profiler, percentile, set_active_profiler


class FakeClock:
    '''A clock the test advances by hand, so frame timings are exact.'''

    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


def run_frames(profiler, clock, count, duration):
    for _ in range(count):
        profiler.begin_frame()
        clock.advance(duration)
        profiler.end_frame()


class PercentileTest(unittest.TestCase):
    def test_empty(self):
        self.assertEqual(percentile([], 0.5), 0.0)

    def test_median_and_extremes(self):
        values = [5, 1, 4, 2, 3]
        self.assertEqual(percentile(values, 0.0), 1)
        self.assertEqual(percentile(values, 0.5), 3)
        self.assertEqual(percentile(values, 1.0), 5)

    def test_p99_picks_the_tail(self):
        values = [1.0] * 95 + [50.0] * 5
        self.assertEqual(percentile(values, 0.99), 50.0)
        self.assertEqual(percentile(values, 0.5), 1.0)

    def test_a_lone_outlier_does_not_move_p99(self):
        # Nearest-rank cannot represent 1-in-100 at p99, so the report has to
        # show max as well or a single stalled frame would go unnoticed.
        values = [1.0] * 99 + [50.0]

        self.assertEqual(percentile(values, 0.99), 1.0)
        self.assertEqual(max(values), 50.0)


class FrameTimingTest(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.profiler = FrameProfiler(clock=self.clock, track_gc=False)

    def test_steady_frames_are_never_long(self):
        run_frames(self.profiler, self.clock, 200, 0.005)

        self.assertEqual(self.profiler.frames, 200)
        self.assertEqual(self.profiler.long_frames, 0)
        self.assertAlmostEqual(self.profiler.median_frame_time, 0.005)

    def test_a_spike_is_counted_as_a_long_frame(self):
        run_frames(self.profiler, self.clock, 200, 0.005)
        run_frames(self.profiler, self.clock, 1, 0.030)

        self.assertEqual(self.profiler.long_frames, 1)

    def test_baseline_needs_a_warmup_before_anything_counts(self):
        # Too few samples to trust a median, so the spike is not judged.
        run_frames(self.profiler, self.clock, FrameProfiler.MIN_SAMPLES_FOR_BASELINE - 1, 0.005)
        run_frames(self.profiler, self.clock, 1, 0.030)

        self.assertEqual(self.profiler.long_frames, 0)

    def test_a_long_frame_does_not_raise_its_own_bar(self):
        # The baseline is sampled before the frame is appended, so a run of slow
        # frames keeps being reported rather than quietly becoming the new normal.
        run_frames(self.profiler, self.clock, 200, 0.005)
        run_frames(self.profiler, self.clock, 5, 0.030)

        self.assertEqual(self.profiler.long_frames, 5)

    def test_end_frame_without_begin_frame_is_ignored(self):
        self.profiler.end_frame()

        self.assertEqual(self.profiler.frames, 0)


class CounterTest(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.profiler = FrameProfiler(clock=self.clock, track_gc=False)

    def test_counters_average_over_frames(self):
        for _ in range(4):
            self.profiler.begin_frame()
            self.profiler.count('chunk_draws', 64)
            self.clock.advance(0.005)
            self.profiler.end_frame()

        self.assertEqual(self.profiler.counters['chunk_draws'], 256)
        self.assertEqual(self.profiler.per_frame('chunk_draws'), 64.0)

    def test_per_frame_of_unknown_counter_is_zero(self):
        self.assertEqual(self.profiler.per_frame('nope'), 0.0)


class GCTrackerTest(unittest.TestCase):
    '''Uses the real clock and real collections -- this measures actual pauses.'''

    def setUp(self):
        self.tracker = GCTracker()
        self.tracker.install()
        self.addCleanup(self.tracker.uninstall)

    def test_a_collection_is_recorded_with_a_nonzero_pause(self):
        gc.collect()

        self.assertGreaterEqual(self.tracker.collections, 1)
        self.assertGreater(self.tracker.total_pause, 0.0)
        self.assertGreater(self.tracker.worst_pause, 0.0)

    def test_generation_is_attributed(self):
        gc.collect(0)

        self.assertGreaterEqual(self.tracker.generation_counts[0], 1)

    def test_uninstall_stops_recording(self):
        self.tracker.uninstall()
        before = self.tracker.collections

        gc.collect()

        self.assertEqual(self.tracker.collections, before)

    def test_reset_window_clears_counts(self):
        gc.collect()
        self.tracker.reset_window()

        self.assertEqual(self.tracker.collections, 0)
        self.assertEqual(self.tracker.total_pause, 0.0)


class GCAttributionTest(unittest.TestCase):
    '''A GC landing inside a frame must be attributed to that frame.

    This is the measurement that justifies doing anything about the GC at all:
    without it, a collection is invisible in the averages.
    '''

    def setUp(self):
        self.clock = FakeClock()
        self.profiler = FrameProfiler(clock=self.clock)
        self.profiler.install()
        self.addCleanup(self.profiler.uninstall)

    def test_frame_containing_a_collection_is_flagged(self):
        self.profiler.begin_frame()
        gc.collect()
        self.clock.advance(0.005)
        self.profiler.end_frame()

        self.assertEqual(self.profiler.frames_with_gc, 1)

    def test_frame_without_a_collection_is_not_flagged(self):
        run_frames(self.profiler, self.clock, 1, 0.005)

        self.assertEqual(self.profiler.frames_with_gc, 0)

    def test_long_frame_caused_by_gc_is_correlated(self):
        run_frames(self.profiler, self.clock, 200, 0.005)

        self.profiler.begin_frame()
        gc.collect()
        self.clock.advance(0.030)
        self.profiler.end_frame()

        self.assertEqual(self.profiler.long_frames, 1)
        self.assertEqual(self.profiler.long_frames_with_gc, 1)


class ReportTest(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.profiler = FrameProfiler(report_interval=5.0, clock=self.clock, track_gc=False)

    def test_report_is_none_before_any_frames(self):
        self.assertIsNone(self.profiler.report())

    def test_report_mentions_frame_rate_and_chunk_waste(self):
        for _ in range(100):
            self.profiler.begin_frame()
            self.profiler.count('chunk_candidates', 64)
            self.profiler.count('chunk_draws', 64)
            self.clock.advance(0.010)
            self.profiler.end_frame()

        text = self.profiler.report()

        self.assertIn('100 frames', text)
        self.assertIn('100.0 fps', text)
        # Nothing is culled yet, so the report should say exactly that.
        self.assertIn('0.0% culled', text)

    def test_maybe_report_waits_for_the_interval(self):
        run_frames(self.profiler, self.clock, 10, 0.010)

        with contextlib.redirect_stdout(io.StringIO()):
            self.assertIsNone(self.profiler.maybe_report())

            self.clock.advance(5.0)
            self.assertIsNotNone(self.profiler.maybe_report())

    def test_reporting_resets_the_window(self):
        run_frames(self.profiler, self.clock, 10, 0.010)
        self.clock.advance(5.0)
        with contextlib.redirect_stdout(io.StringIO()):
            self.profiler.maybe_report()

        self.assertEqual(self.profiler.frames, 0)
        self.assertEqual(self.profiler.total_frames, 10)


class ActiveProfilerTest(unittest.TestCase):
    def tearDown(self):
        set_active_profiler(None)

    def test_install_and_uninstall_toggle_the_active_profiler(self):
        profiler = FrameProfiler(clock=FakeClock())

        self.assertIsNone(active_profiler())

        profiler.install()
        self.assertIs(active_profiler(), profiler)

        profiler.uninstall()
        self.assertIsNone(active_profiler())


if __name__ == '__main__':
    unittest.main()
