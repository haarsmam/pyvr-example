'''
Garbage collector control.

CPython frees most objects by reference counting, which is incremental and
harmless. The cycle collector is the problem: it stops the world, and it runs
whenever allocation counters happen to trip -- which in a render loop means at
an arbitrary point inside an arbitrary frame. A collection that overruns the
display period is not a dropped frame in VR, it is a reprojection artifact the
user feels in their inner ear.

Two things are done about that:

  freeze_baseline()  After world generation, everything alive is the permanent
                     heap: modules, shaders, the block cache, chunk meshes. That
                     is by far the largest set of objects in the process and
                     none of it will ever become garbage, because the world is
                     built entirely during startup and no block is added or
                     removed after that. gc.freeze() moves it to a generation
                     the collector never examines again, so every later pass has
                     a small heap to walk instead of the whole world.

  step()             With automatic collection off, collections happen at a
                     chosen point -- the end of a frame, after submit -- and
                     mostly at generation 0, which only walks what was allocated
                     since the last pass. This trades rare unpredictable stalls
                     for frequent tiny ones, which is the right trade when the
                     deadline is 13.8ms and missing it is felt rather than seen.

Both are disabled together by constructing ManagedGC(enabled=False), so the
stock behaviour can be measured against this one.
'''

import gc


class GCSchedule:
    '''Decides which generation, if any, to collect on a given frame.

    Pure arithmetic, so the policy can be tested without invoking the real
    collector. Intervals are in frames; 0 disables that generation entirely.

    Higher generations subsume lower ones (collecting gen2 also collects gen0
    and gen1), so when several intervals land on the same frame the highest one
    wins and the others are skipped rather than run separately.
    '''

    def __init__(self, gen0_interval=6, gen1_interval=120, gen2_interval=1800):
        self.gen0_interval = gen0_interval
        self.gen1_interval = gen1_interval
        self.gen2_interval = gen2_interval

    def generation_for_frame(self, frame_index):
        # Frame 0 collects nothing: freeze_baseline() has just run.
        if frame_index <= 0:
            return None

        for generation, interval in ((2, self.gen2_interval), (1, self.gen1_interval), (0, self.gen0_interval)):
            if interval and (frame_index % interval == 0):
                return generation

        return None


class ManagedGC:
    '''Owns the process-wide GC configuration for the lifetime of the run.'''

    def __init__(self, schedule=None, enabled=True, collect=gc.collect):
        self.schedule = schedule if schedule else GCSchedule()
        self.enabled = enabled
        self._collect = collect

        self.collections = [0, 0, 0]
        self.frozen_objects = 0

        self._applied = False
        self._was_enabled = gc.isenabled()

    def freeze_baseline(self):
        '''Freeze the startup heap and take over collection scheduling.

        Returns the number of objects frozen, or 0 when disabled.

        Only safe because nothing frozen here can later become garbage. If the
        world ever gains runtime block editing, the chunk/block cycles created
        during startup would become unreachable while still frozen, and would
        never be reclaimed -- revisit this before adding that.
        '''
        if not self.enabled:
            return 0

        self._was_enabled = gc.isenabled()

        # CPython freezes a few hundred objects of its own during interpreter
        # startup, so the count has to be read as a delta to mean anything.
        already_frozen = gc.get_freeze_count()

        gc.collect()
        gc.freeze()
        gc.disable()

        self._applied = True
        self.frozen_objects = gc.get_freeze_count() - already_frozen

        return self.frozen_objects

    def restore(self):
        '''Put the collector back the way it was found.

        gc.unfreeze() is all-or-nothing, so this also thaws the objects CPython
        froze during startup. Harmless -- they simply become collectable again.
        '''
        if not self._applied:
            return

        gc.unfreeze()
        if self._was_enabled:
            gc.enable()

        self._applied = False

    def step(self, frame_index):
        '''Run any collection this frame is due for. Returns the generation, or None.'''
        if not self.enabled:
            return None

        generation = self.schedule.generation_for_frame(frame_index)
        if generation is None:
            return None

        self._collect(generation)
        self.collections[generation] += 1

        return generation
