'''Tests for frustum culling of chunks.

The risk here is a silently transposed matrix: the wrong convention still
produces a plausible-looking set of planes and simply culls the wrong things.
So the frustum test is cross-checked against projecting points through the same
matrix and applying OpenGL's own clip-space rule, and that reference is itself
anchored against known camera conventions (down -Z, +X right, +Y up) so both
cannot be wrong in the same direction.

Matrices are built through the real pyopenxr path used by mgllib/xrwin.py.

Run from the repository root:

    python -m unittest discover -s tests -t .
'''

import math
import os
import sys
import types
import unittest

import glm
import numpy as np
import xr

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mgllib.world.chunk import Chunk
from mgllib.world.const import BLOCK_SCALE, CHUNK_SIZE
from mgllib.world.culling import SWAY_MARGIN, ChunkCuller, frustum_planes, visible_mask


def make_view_projection(position=(0.0, 0.0, 0.0), orientation=(0.0, 0.0, 0.0, 1.0), half_fov_degrees=45.0, near=0.03, far=200.0):
    '''Build a view-projection exactly the way XRWindow.run does.'''
    angle = math.radians(half_fov_degrees)
    fov = xr.Fovf(angle_left=-angle, angle_right=angle, angle_up=angle, angle_down=-angle)

    projection = xr.Matrix4x4f.create_projection_fov(graphics_api=xr.GraphicsAPI.OPENGL, fov=fov, near_z=near, far_z=far)
    to_view = xr.Matrix4x4f.create_translation_rotation_scale(
        translation=xr.Vector3f(*position), rotation=xr.Quaternionf(*orientation), scale=xr.Vector3f(1.0, 1.0, 1.0)
    )
    new_view = xr.Matrix4x4f.invert_rigid_body(to_view)

    # Keep a reference: Matrix4x4f.as_numpy() hands back a view built with
    # ctypes from_address, which does not keep the matrix alive. Reading it off
    # a temporary yields freed memory.
    combined = projection @ new_view

    return np.array(combined.as_numpy(), dtype=np.float64)


def apply_world_transform(view_projection, player_pos=(0.0, 0.0, 0.0), player_yaw=0.0, head=(0.0, 0.0, 0.0)):
    '''Reproduce XRCamera.cycle's prepped_matrix for a given player pose.'''
    player_matrix = np.array((glm.translate(glm.vec3(player_pos)) * glm.rotate(player_yaw, glm.vec3(0, 1, 0))).to_list()).T
    world_matrix = np.linalg.inv(player_matrix)
    head_transform = np.array(glm.translate(glm.vec3(head)).to_list())

    return (world_matrix.T @ head_transform @ np.reshape(view_projection, (4, 4))).flatten()


def clip_coords(view_projection, point):
    '''Project a world point the way the vertex shader does.

    The uniform is a flat column-major mat4, so reshaping row-major and
    transposing recovers the matrix that multiplies a column vector.
    '''
    matrix = np.asarray(view_projection, dtype=np.float64).reshape(4, 4).T
    return matrix @ np.array([point[0], point[1], point[2], 1.0])


def projects_inside(view_projection, point):
    '''OpenGL's clip-space visibility rule: -w <= x, y, z <= w, with w > 0.'''
    clip = clip_coords(view_projection, point)
    w = clip[3]
    if w <= 0.0:
        return False
    return bool(np.all(clip[:3] >= -w) and np.all(clip[:3] <= w))


def box(point):
    '''A degenerate AABB, so the conservative box test becomes an exact point test.'''
    array = np.array([point], dtype=np.float64)
    return array, array.copy()


class FakeChunk:
    def __init__(self, low, high):
        self.world_bounds = (low, high)

    def __repr__(self):
        return 'FakeChunk({0}, {1})'.format(*self.world_bounds)


class FakeCamera:
    def __init__(self, prepped_matrix):
        self.prepped_matrix = prepped_matrix


class ProjectionReferenceTest(unittest.TestCase):
    '''Anchors the reference implementation against known camera conventions.

    If the matrix convention were transposed, these would fail -- which is what
    stops the cross-validation below from being circular.
    '''

    def setUp(self):
        self.view_projection = make_view_projection()

    def test_point_ahead_is_in_front_and_centred(self):
        clip = clip_coords(self.view_projection, (0.0, 0.0, -5.0))

        self.assertGreater(clip[3], 0.0)
        self.assertAlmostEqual(clip[0], 0.0, places=6)
        self.assertAlmostEqual(clip[1], 0.0, places=6)

    def test_point_behind_the_camera_has_negative_w(self):
        # The camera looks down -Z, so +Z is behind it.
        self.assertLess(clip_coords(self.view_projection, (0.0, 0.0, 5.0))[3], 0.0)

    def test_right_of_camera_projects_right(self):
        self.assertGreater(clip_coords(self.view_projection, (1.0, 0.0, -5.0))[0], 0.0)

    def test_above_camera_projects_up(self):
        self.assertGreater(clip_coords(self.view_projection, (0.0, 1.0, -5.0))[1], 0.0)


class FrustumPlaneTest(unittest.TestCase):
    def setUp(self):
        self.view_projection = make_view_projection()
        self.planes = frustum_planes(self.view_projection)

    def test_six_normalised_planes(self):
        self.assertEqual(self.planes.shape, (6, 4))
        np.testing.assert_allclose(np.linalg.norm(self.planes[:, :3], axis=1), np.ones(6))

    def test_point_ahead_is_visible(self):
        self.assertTrue(visible_mask(self.planes, *box((0.0, 0.0, -5.0)))[0])

    def test_point_behind_is_culled(self):
        self.assertFalse(visible_mask(self.planes, *box((0.0, 0.0, 5.0)))[0])

    def test_point_beyond_the_far_plane_is_culled(self):
        self.assertFalse(visible_mask(self.planes, *box((0.0, 0.0, -500.0)))[0])

    def test_point_far_off_to_the_side_is_culled(self):
        self.assertFalse(visible_mask(self.planes, *box((100.0, 0.0, -5.0)))[0])

    def test_box_straddling_the_view_edge_is_kept(self):
        # Conservative: a box only partly on screen must still be drawn.
        mins = np.array([[-100.0, -1.0, -6.0]])
        maxes = np.array([[0.0, 1.0, -4.0]])

        self.assertTrue(visible_mask(self.planes, mins, maxes)[0])

    def test_empty_input(self):
        mask = visible_mask(self.planes, np.zeros((0, 3)), np.zeros((0, 3)))

        self.assertEqual(len(mask), 0)


class CrossValidationTest(unittest.TestCase):
    '''The frustum test must agree with projecting the point through the matrix.

    Run for several camera and player poses, so a convention error that happens
    to cancel out at the origin cannot survive.
    '''

    POSES = [
        ('identity', {}),
        ('yawed', {'player_yaw': 0.9}),
        ('translated', {'player_pos': (12.0, 3.0, -7.0)}),
        ('translated and yawed', {'player_pos': (-20.0, 1.5, 8.0), 'player_yaw': -2.1}),
        ('head offset', {'player_pos': (4.0, 0.0, 4.0), 'player_yaw': 0.4, 'head': (0.3, 0.0, -0.2)}),
    ]

    def sample_points(self):
        random = np.random.RandomState(20260919)
        return random.uniform(-40.0, 40.0, size=(400, 3))

    def test_mask_matches_clip_space_for_every_pose(self):
        for label, kwargs in self.POSES:
            with self.subTest(pose=label):
                view_projection = apply_world_transform(make_view_projection(), **kwargs)
                planes = frustum_planes(view_projection)

                inside_count = 0
                outside_count = 0

                for point in self.sample_points():
                    clip = clip_coords(view_projection, point)

                    # Skip points sitting on a boundary, where the two
                    # formulations differ only by floating point noise.
                    if abs(clip[3]) < 1e-3 or np.any(np.abs(np.abs(clip[:3]) - abs(clip[3])) < 1e-3):
                        continue

                    expected = projects_inside(view_projection, point)
                    actual = bool(visible_mask(planes, *box(point))[0])

                    self.assertEqual(actual, expected, '{0} at {1}'.format(label, point))

                    inside_count += expected
                    outside_count += not expected

                # Guard against a vacuous pass.
                self.assertGreater(inside_count, 0, label)
                self.assertGreater(outside_count, 0, label)


class MarginTest(unittest.TestCase):
    def setUp(self):
        self.planes = frustum_planes(make_view_projection())

    def test_margin_keeps_boxes_just_outside_the_frustum(self):
        # Grass and trees are displaced by the vertex shader after culling has
        # already decided, so the margin has to absorb that movement.
        just_behind = box((0.0, 0.0, 0.1))

        self.assertFalse(visible_mask(self.planes, *just_behind, margin=0.0)[0])
        self.assertTrue(visible_mask(self.planes, *just_behind, margin=SWAY_MARGIN)[0])


class ChunkCullerTest(unittest.TestCase):
    def setUp(self):
        self.camera = FakeCamera(make_view_projection())
        self.ahead = FakeChunk((-2.0, -2.0, -12.0), (2.0, 2.0, -8.0))
        self.behind = FakeChunk((-2.0, -2.0, 8.0), (2.0, 2.0, 12.0))
        self.culler = ChunkCuller()

    def test_selects_only_what_is_in_view(self):
        selected = self.culler.select([self.ahead, self.behind], self.camera)

        self.assertEqual(selected, [self.ahead])

    def test_disabled_submits_everything(self):
        culler = ChunkCuller(enabled=False)

        selected = culler.select([self.ahead, self.behind], self.camera)

        self.assertEqual(selected, [self.ahead, self.behind])

    def test_missing_matrix_falls_back_to_submitting_everything(self):
        # First frame, before XRCamera.cycle has run: draw too much rather than
        # nothing at all.
        selected = self.culler.select([self.ahead, self.behind], FakeCamera(None))

        self.assertEqual(selected, [self.ahead, self.behind])

    def test_missing_camera_falls_back_to_submitting_everything(self):
        selected = self.culler.select([self.ahead, self.behind], None)

        self.assertEqual(selected, [self.ahead, self.behind])

    def test_empty_world(self):
        self.assertEqual(self.culler.select([], self.camera), [])

    def test_accepts_a_dict_values_view(self):
        chunks = {0: self.ahead, 1: self.behind}

        self.assertEqual(self.culler.select(chunks.values(), self.camera), [self.ahead])

    def test_turning_around_changes_the_selection(self):
        behind_camera = FakeCamera(apply_world_transform(make_view_projection(), player_yaw=math.pi))

        self.assertEqual(self.culler.select([self.ahead, self.behind], self.camera), [self.ahead])
        self.assertEqual(self.culler.select([self.ahead, self.behind], behind_camera), [self.behind])

    def test_bounds_are_cached_between_calls(self):
        class CountingChunk(FakeChunk):
            reads = 0

            @property
            def world_bounds(self):
                type(self).reads += 1
                return self._bounds

            @world_bounds.setter
            def world_bounds(self, value):
                self._bounds = value

        chunk = CountingChunk((-2.0, -2.0, -12.0), (2.0, 2.0, -8.0))

        for _ in range(10):
            self.culler.select([chunk], self.camera)

        self.assertEqual(CountingChunk.reads, 1)

    def test_invalidate_forces_a_rebuild(self):
        self.culler.select([self.ahead], self.camera)

        # Decor added after the first frame would move this chunk's bounds.
        self.ahead.world_bounds = ((-2.0, -2.0, 8.0), (2.0, 2.0, 12.0))
        self.culler.invalidate()

        self.assertEqual(self.culler.select([self.ahead], self.camera), [])

    def test_a_new_chunk_is_picked_up_without_an_explicit_invalidate(self):
        self.culler.select([self.ahead], self.camera)

        selected = self.culler.select([self.ahead, self.behind], self.camera)

        self.assertEqual(selected, [self.ahead])


class CullingBenefitTest(unittest.TestCase):
    '''The point of the exercise: most of a surrounding world is not on screen.'''

    def test_most_of_a_surrounding_grid_is_culled(self):
        extent = 12.0
        chunks = []
        for x in range(-4, 4):
            for z in range(-4, 4):
                low = (x * extent, -extent, z * extent)
                high = (low[0] + extent, extent, low[2] + extent)
                chunks.append(FakeChunk(low, high))

        selected = ChunkCuller().select(chunks, FakeCamera(make_view_projection()))

        self.assertLess(len(selected), len(chunks) / 2)
        self.assertGreater(len(selected), 0)


class ChunkBoundsTest(unittest.TestCase):
    """The bounds culling actually tests against, on the real Chunk class.

    Chunk needs no GL context to compute these -- only its parent's program
    reference, which is never dereferenced here.
    """

    EXTENT = CHUNK_SIZE * BLOCK_SCALE

    def make_chunk(self, chunk_id=(0, 0, 0)):
        return Chunk(types.SimpleNamespace(program=None), chunk_id)

    def make_decor(self, low, high, name='tree'):
        return types.SimpleNamespace(source=types.SimpleNamespace(name=name), bounds=(low, high))

    def test_bounds_of_a_plain_chunk_are_its_cube(self):
        chunk = self.make_chunk((1, 0, -2))

        low, high = chunk.world_bounds

        self.assertEqual(low, (self.EXTENT, 0.0, -2 * self.EXTENT))
        self.assertEqual(high, (2 * self.EXTENT, self.EXTENT, -1 * self.EXTENT))

    def test_bounds_grow_to_cover_a_tree_reaching_above_the_chunk(self):
        # The case that would pop visibly: a tree owned by this chunk because
        # its base sits here, with its canopy well outside the cube.
        chunk = self.make_chunk((0, 0, 0))
        canopy_top = self.EXTENT + 6.0

        chunk.add_decor(self.make_decor((-3.0, 0.0, -3.0), (4.0, canopy_top, 4.0)))
        low, high = chunk.world_bounds

        self.assertEqual(low, (-3.0, 0.0, -3.0))
        self.assertEqual(high, (self.EXTENT, canopy_top, self.EXTENT))

    def test_decor_inside_the_cube_does_not_change_the_bounds(self):
        chunk = self.make_chunk((0, 0, 0))
        before = chunk.world_bounds

        chunk.add_decor(self.make_decor((1.0, 1.0, 1.0), (2.0, 2.0, 2.0), name='grass'))

        self.assertEqual(chunk.world_bounds, before)

    def test_adding_decor_invalidates_cached_bounds(self):
        chunk = self.make_chunk((0, 0, 0))

        # Read first, so a stale cache would be caught.
        self.assertEqual(chunk.world_bounds[0], (0.0, 0.0, 0.0))

        chunk.add_decor(self.make_decor((-5.0, 0.0, 0.0), (1.0, 1.0, 1.0)))

        self.assertEqual(chunk.world_bounds[0][0], -5.0)

    def test_bounds_are_cached(self):
        chunk = self.make_chunk((0, 0, 0))

        self.assertIs(chunk.world_bounds, chunk.world_bounds)


if __name__ == '__main__':
    unittest.main()
