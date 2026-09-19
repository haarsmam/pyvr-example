'''Tests for garbage collector scheduling.

The schedule is tested against a recording collector rather than the real one,
so the policy is verified without the test's own allocations perturbing it.

Run from the repository root:

    python -m unittest discover -s tests -t .
'''

import gc
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mgllib.gc_control import GCSchedule, ManagedGC


class RecordingCollector:
    '''Stands in for gc.collect so tests observe the schedule, not the heap.'''

    def __init__(self):
        self.calls = []

    def __call__(self, generation):
        self.calls.append(generation)


class GCScheduleTest(unittest.TestCase):
    def setUp(self):
        self.schedule = GCSchedule(gen0_interval=6, gen1_interval=120, gen2_interval=1800)

    def test_frame_zero_collects_nothing(self):
        # freeze_baseline() has just collected; doing it again would be waste.
        self.assertIsNone(self.schedule.generation_for_frame(0))

    def test_ordinary_frames_collect_nothing(self):
        for frame in (1, 2, 3, 4, 5, 7, 11):
            self.assertIsNone(self.schedule.generation_for_frame(frame), frame)

    def test_gen0_runs_on_its_interval(self):
        for frame in (6, 12, 18, 66):
            self.assertEqual(self.schedule.generation_for_frame(frame), 0, frame)

    def test_gen1_runs_on_its_interval(self):
        for frame in (120, 240, 600):
            self.assertEqual(self.schedule.generation_for_frame(frame), 1, frame)

    def test_gen2_runs_on_its_interval(self):
        for frame in (1800, 3600):
            self.assertEqual(self.schedule.generation_for_frame(frame), 2, frame)

    def test_highest_generation_wins_when_intervals_coincide(self):
        # 1800 is a multiple of all three intervals. Collecting gen2 already
        # collects gen0 and gen1, so it must not also schedule them.
        self.assertEqual(self.schedule.generation_for_frame(1800), 2)
        self.assertEqual(self.schedule.generation_for_frame(120), 1)

    def test_zero_interval_disables_a_generation(self):
        schedule = GCSchedule(gen0_interval=6, gen1_interval=0, gen2_interval=0)

        self.assertEqual(schedule.generation_for_frame(120), 0)
        self.assertEqual(schedule.generation_for_frame(1800), 0)

    def test_gen0_dominates_the_frame_budget_over_a_long_run(self):
        # The point of the policy: many cheap passes, very few expensive ones.
        counts = {0: 0, 1: 0, 2: 0, None: 0}
        for frame in range(1, 3601):
            counts[self.schedule.generation_for_frame(frame)] += 1

        self.assertEqual(counts[2], 2)
        self.assertEqual(counts[1], 28)
        self.assertEqual(counts[0], 570)


class ManagedGCStepTest(unittest.TestCase):
    def setUp(self):
        self.collector = RecordingCollector()
        self.managed = ManagedGC(schedule=GCSchedule(gen0_interval=3, gen1_interval=9, gen2_interval=0), collect=self.collector)

    def test_step_collects_on_scheduled_frames_only(self):
        for frame in range(10):
            self.managed.step(frame)

        self.assertEqual(self.collector.calls, [0, 0, 1])

    def test_step_reports_the_generation_it_ran(self):
        self.assertIsNone(self.managed.step(1))
        self.assertEqual(self.managed.step(3), 0)
        self.assertEqual(self.managed.step(9), 1)

    def test_counts_are_tallied_per_generation(self):
        for frame in range(19):
            self.managed.step(frame)

        self.assertEqual(self.managed.collections, [4, 2, 0])

    def test_disabled_never_collects(self):
        managed = ManagedGC(enabled=False, collect=self.collector)

        for frame in range(100):
            self.assertIsNone(managed.step(frame))

        self.assertEqual(self.collector.calls, [])


class FreezeBaselineTest(unittest.TestCase):
    '''Touches the real collector, so every test restores it.'''

    def tearDown(self):
        gc.unfreeze()
        gc.enable()

    def test_freeze_disables_automatic_collection_and_freezes_objects(self):
        managed = ManagedGC()
        self.addCleanup(managed.restore)
        before = gc.get_freeze_count()

        frozen = managed.freeze_baseline()

        self.assertGreater(frozen, 0)
        self.assertFalse(gc.isenabled())
        self.assertEqual(gc.get_freeze_count(), before + frozen)

    def test_reported_count_excludes_objects_frozen_before_the_call(self):
        # CPython freezes objects of its own at startup; counting those as ours
        # would make the startup log meaningless. Freeze explicitly here rather
        # than relying on the interpreter's own frozen set surviving other tests.
        gc.freeze()
        pre_existing = gc.get_freeze_count()
        self.assertGreater(pre_existing, 0)

        managed = ManagedGC()
        self.addCleanup(managed.restore)

        frozen = managed.freeze_baseline()

        self.assertLess(frozen, pre_existing)
        self.assertEqual(gc.get_freeze_count(), pre_existing + frozen)

    def test_restore_puts_the_collector_back(self):
        self.assertTrue(gc.isenabled())

        managed = ManagedGC()
        managed.freeze_baseline()
        managed.restore()

        self.assertTrue(gc.isenabled())
        self.assertEqual(gc.get_freeze_count(), 0)

    def test_objects_created_after_the_freeze_are_still_collectable(self):
        # The freeze must only cover the startup heap. Anything allocated later
        # -- tracers, particles, NPCs -- has to stay reclaimable.
        managed = ManagedGC()
        self.addCleanup(managed.restore)
        managed.freeze_baseline()

        cycle = {}
        cycle['self'] = cycle
        del cycle

        collected = gc.collect()

        self.assertGreaterEqual(collected, 1)

    def test_disabled_leaves_the_collector_untouched(self):
        managed = ManagedGC(enabled=False)
        before = gc.get_freeze_count()

        self.assertEqual(managed.freeze_baseline(), 0)
        self.assertTrue(gc.isenabled())
        self.assertEqual(gc.get_freeze_count(), before)

    def test_restore_without_freeze_is_harmless(self):
        ManagedGC().restore()

        self.assertTrue(gc.isenabled())


if __name__ == '__main__':
    unittest.main()
