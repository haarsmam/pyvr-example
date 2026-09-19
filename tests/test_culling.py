'''Characterisation tests for chunk visibility selection.

These describe what the renderer does *today*, not what it should do. Every
chunk is submitted for every eye regardless of where the camera looks, so the
assertions below record that waste rather than approving of it. Phase 3 replaces
them with assertions that off-screen chunks are dropped.

Run from the repository root:

    python -m unittest discover -s tests -t .
'''

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mgllib.world.culling import select_visible


class FakeChunk:
    '''Stands in for a Chunk so no GL context is needed.'''

    def __init__(self, chunk_id):
        self.chunk_id = chunk_id

    def __repr__(self):
        return 'FakeChunk{0}'.format(self.chunk_id)


def grid_of_chunks(radius=4):
    return [FakeChunk((x, 0, z)) for x in range(-radius, radius) for z in range(-radius, radius)]


class SelectVisibleTest(unittest.TestCase):
    def test_nothing_is_culled(self):
        chunks = grid_of_chunks()

        selected = select_visible(chunks, camera=None)

        self.assertEqual(len(selected), len(chunks))
        self.assertEqual(set(id(c) for c in selected), set(id(c) for c in chunks))

    def test_chunks_behind_the_camera_are_still_submitted(self):
        # The waste this measures: with a ~100 degree horizontal FOV, well under
        # half a surrounding grid can possibly be on screen, yet all of it is
        # drawn -- twice, once per eye.
        chunks = grid_of_chunks()

        self.assertEqual(len(select_visible(chunks, camera=None)), 64)

    def test_accepts_a_dict_values_view(self):
        # World.chunks is a dict; select_visible is handed its .values().
        chunks = {chunk.chunk_id: chunk for chunk in grid_of_chunks()}

        self.assertEqual(len(select_visible(chunks.values(), camera=None)), 64)

    def test_returns_a_list_not_a_live_view(self):
        # The render loop must not be iterating a view that world edits mutate.
        chunks = {chunk.chunk_id: chunk for chunk in grid_of_chunks()}

        selected = select_visible(chunks.values(), camera=None)
        chunks[(99, 0, 99)] = FakeChunk((99, 0, 99))

        self.assertEqual(len(selected), 64)

    def test_empty_world(self):
        self.assertEqual(select_visible([], camera=None), [])


if __name__ == '__main__':
    unittest.main()
