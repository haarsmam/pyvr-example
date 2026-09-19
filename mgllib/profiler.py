'''
Frame instrumentation.

Exists to answer two questions with numbers instead of intuition:

  1. Is the garbage collector stalling frames? A collection that lands mid-frame
     costs that frame, and a missed frame in VR is a reprojection artifact the
     user feels rather than sees. Mean frame time hides this entirely; it only
     shows up in the tail, so the tail is what gets reported.
  2. How much geometry is being submitted that the eye cannot see? The world
     submits every chunk for every eye with no visibility test at all.

Nothing here changes behaviour. It only measures.
'''

import gc
import math
import time
from collections import deque

# The profiler the renderer should report into, or None when not profiling.
# Module-level rather than routed through mgllib.elements so that the render
# path stays decoupled and the whole module is importable without a GL context.
_ACTIVE = None


def active_profiler():
    return _ACTIVE


def set_active_profiler(profiler):
    global _ACTIVE
    _ACTIVE = profiler


def percentile(values, fraction):
    '''Nearest-rank percentile, `fraction` in 0..1.

    Note that with only N samples a single outlier cannot move the (1 - 1/N)
    percentile -- that is why the report carries max alongside p99.
    '''
    if not values:
        return 0.0
    ordered = sorted(values)
    index = math.ceil(fraction * len(ordered)) - 1
    return ordered[min(len(ordered) - 1, max(0, index))]


class GCTracker:
    '''Measures how long each garbage collection stalls the main thread.

    Uses gc.callbacks, which fires synchronously around every collection, so the
    span between 'start' and 'stop' is exactly the time the interpreter spent
    collecting instead of rendering.
    '''

    def __init__(self, history=512, clock=time.perf_counter):
        self._clock = clock

        self.pauses = deque(maxlen=history)
        self.generation_counts = [0, 0, 0]
        self.collections = 0
        self.total_pause = 0.0
        self.worst_pause = 0.0

        self._pause_start = None
        self._installed = False

    @property
    def installed(self):
        return self._installed

    def install(self):
        if not self._installed:
            gc.callbacks.append(self._on_gc)
            self._installed = True

    def uninstall(self):
        if self._installed:
            gc.callbacks.remove(self._on_gc)
            self._installed = False

    def _on_gc(self, phase, info):
        if phase == 'start':
            self._pause_start = self._clock()
        elif self._pause_start is not None:
            duration = self._clock() - self._pause_start
            self._pause_start = None

            generation = info.get('generation', 0)
            if 0 <= generation < len(self.generation_counts):
                self.generation_counts[generation] += 1

            self.collections += 1
            self.total_pause += duration
            self.worst_pause = max(self.worst_pause, duration)
            self.pauses.append((generation, duration))

    def reset_window(self):
        self.generation_counts = [0, 0, 0]
        self.collections = 0
        self.total_pause = 0.0
        self.worst_pause = 0.0


class FrameProfiler:
    '''Per-frame timing, GC attribution and render counters.

    A frame counts as "long" when it exceeds LONG_FRAME_FACTOR times the median
    of the frames before it. Calibrating against the observed median rather than
    a fixed budget means this works without knowing the headset's refresh rate.
    '''

    LONG_FRAME_FACTOR = 1.5
    MIN_SAMPLES_FOR_BASELINE = 30

    def __init__(self, report_interval=5.0, window=2048, clock=time.perf_counter, track_gc=True):
        self.report_interval = report_interval
        self._clock = clock

        self.frame_times = deque(maxlen=window)
        self.counters = {}

        self.frames = 0
        self.long_frames = 0
        self.frames_with_gc = 0
        self.long_frames_with_gc = 0
        self.total_frames = 0

        self.gc_tracker = GCTracker(clock=time.perf_counter) if track_gc else None

        self._frame_start = None
        self._gc_count_at_frame_start = 0
        self._window_start = self._clock()
        self._last_report = self._window_start

    def install(self):
        if self.gc_tracker:
            self.gc_tracker.install()
        set_active_profiler(self)

    def uninstall(self):
        if self.gc_tracker:
            self.gc_tracker.uninstall()
        if active_profiler() is self:
            set_active_profiler(None)

    @property
    def median_frame_time(self):
        return percentile(self.frame_times, 0.5)

    def begin_frame(self):
        self._frame_start = self._clock()
        self._gc_count_at_frame_start = self.gc_tracker.collections if self.gc_tracker else 0

    def end_frame(self):
        if self._frame_start is None:
            return

        duration = self._clock() - self._frame_start
        self._frame_start = None

        # Baseline is taken before appending so a long frame cannot raise the bar
        # it is being measured against.
        baseline = self.median_frame_time if len(self.frame_times) >= self.MIN_SAMPLES_FOR_BASELINE else 0.0

        collected = self.gc_tracker and (self.gc_tracker.collections > self._gc_count_at_frame_start)

        self.frame_times.append(duration)
        self.frames += 1
        self.total_frames += 1

        if collected:
            self.frames_with_gc += 1

        if baseline and (duration > baseline * self.LONG_FRAME_FACTOR):
            self.long_frames += 1
            if collected:
                self.long_frames_with_gc += 1

    def count(self, name, amount=1):
        self.counters[name] = self.counters.get(name, 0) + amount

    def per_frame(self, name):
        if not self.frames:
            return 0.0
        return self.counters.get(name, 0) / self.frames

    def report(self):
        '''Format the window's statistics, or None if no frames were recorded.'''
        if not self.frames:
            return None

        elapsed = max(self._clock() - self._window_start, 1e-9)
        times_ms = [t * 1000.0 for t in self.frame_times]

        lines = [
            '[PERF] {0} frames / {1:.1f}s ({2:.1f} fps) | frame ms: med {3:.2f} p99 {4:.2f} max {5:.2f} | long {6} ({7:.1f}%)'.format(
                self.frames,
                elapsed,
                self.frames / elapsed,
                percentile(times_ms, 0.5),
                percentile(times_ms, 0.99),
                max(times_ms) if times_ms else 0.0,
                self.long_frames,
                100.0 * self.long_frames / self.frames,
            )
        ]

        if self.gc_tracker:
            tracker = self.gc_tracker
            lines.append(
                '[PERF]   gc: {0} collections (gen0 {1} / gen1 {2} / gen2 {3}) | pause ms: total {4:.1f} worst {5:.2f} | {6:.2f}% of wall | {7} frames hit gc, {8} of those ran long'.format(
                    tracker.collections,
                    tracker.generation_counts[0],
                    tracker.generation_counts[1],
                    tracker.generation_counts[2],
                    tracker.total_pause * 1000.0,
                    tracker.worst_pause * 1000.0,
                    100.0 * tracker.total_pause / elapsed,
                    self.frames_with_gc,
                    self.long_frames_with_gc,
                )
            )

        candidates = self.per_frame('chunk_candidates')
        if candidates:
            drawn = self.per_frame('chunk_draws')
            lines.append(
                '[PERF]   chunks/frame (both eyes): {0:.1f} drawn of {1:.1f} candidates ({2:.1f}% culled)'.format(
                    drawn, candidates, 100.0 * (1.0 - drawn / candidates)
                )
            )

        return '\n'.join(lines)

    def reset_window(self):
        now = self._clock()

        self.frame_times.clear()
        self.counters = {}

        self.frames = 0
        self.long_frames = 0
        self.frames_with_gc = 0
        self.long_frames_with_gc = 0

        if self.gc_tracker:
            self.gc_tracker.reset_window()

        self._window_start = now
        self._last_report = now

    def maybe_report(self):
        '''Print and reset the window if the report interval has elapsed.'''
        now = self._clock()
        if (now - self._last_report) < self.report_interval:
            return None

        text = self.report()
        if text:
            print(text, flush=True)

        self.reset_window()
        return text
