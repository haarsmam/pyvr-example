'''
Chunk visibility selection.

Every chunk used to be submitted for every eye, every frame, including chunks
behind the head. This selects the ones that can actually intersect the view
frustum.

Two details make this less trivial than a cube-versus-frustum test:

  * Block geometry is chunk-local and placed by the chunk's world_transform, so
    a chunk's blocks are bounded exactly by its own cube. Decor is not: Decor
    bakes its vertices into world space at build time and decor.vert/grass.vert/
    tree.vert ignore world_transform entirely. A tree owned by a chunk because
    its base sits there extends well above and beside that chunk's cube, so
    bounds have to come from the geometry rather than the grid.

  * The test runs twice per frame, once per eye. Doing it per chunk in Python
    would cost more than the draw calls it saves, so bounds are kept as stacked
    arrays and the whole world is tested in a handful of numpy operations.

The frustum planes come from the same matrix the vertex shaders receive, so the
test cannot drift from what is actually rendered.
'''

import numpy as np

# Grass and trees are displaced in the vertex shader by a fraction of their
# height, which their baked bounds do not account for.
SWAY_MARGIN = 0.25

# Plane order is arbitrary; a box has to pass all six either way.
PLANE_COUNT = 6


def frustum_planes(view_projection):
    '''Extract the six frustum planes from a view-projection matrix.

    `view_projection` is the flat 16-float, column-major matrix handed to the
    shaders as the `view_projection` uniform. Returns a (6, 4) array of
    (a, b, c, d) planes, normalised, oriented so that a point is inside the
    frustum when a*x + b*y + c*z + d >= 0 for every plane.

    This is the Gribb-Hartmann method: clip space is defined by -w <= x,y,z <= w,
    and each of those six inequalities is a plane in world space once the rows of
    the matrix are added to or subtracted from the w row.
    '''
    matrix = np.asarray(view_projection, dtype=np.float64).reshape(4, 4).T

    row_x, row_y, row_z, row_w = matrix[0], matrix[1], matrix[2], matrix[3]

    planes = np.stack([row_w + row_x, row_w - row_x, row_w + row_y, row_w - row_y, row_w + row_z, row_w - row_z])

    # Normalising by the normal's length turns the plane equation into a signed
    # distance, which keeps the margin below meaningful in world units.
    lengths = np.linalg.norm(planes[:, :3], axis=1)
    lengths[lengths == 0.0] = 1.0

    return planes / lengths[:, None]


def visible_mask(planes, mins, maxes, margin=0.0):
    '''Boolean mask of which boxes intersect the frustum.

    `mins` and `maxes` are (N, 3). A box is rejected only when it lies entirely
    outside a single plane, tested via the box corner furthest along that plane's
    normal. That is the standard conservative test: it never culls something
    visible, but may keep a box near a frustum corner that is in fact outside.
    '''
    count = len(mins)
    if not count:
        return np.zeros(0, dtype=bool)

    normals = planes[:, :3]
    offsets = planes[:, 3]

    # For each plane, the corner of each box furthest along that plane's normal.
    positive = np.where(normals[:, None, :] >= 0.0, maxes[None, :, :], mins[None, :, :])

    distances = np.einsum('pi,pni->pn', normals, positive) + offsets[:, None]

    return np.all(distances >= -margin, axis=0)


class ChunkCuller:
    '''Selects visible chunks, caching the world's bounds between frames.

    Bounds only change when the world is edited, which in this demo happens
    during generation and never afterwards, so the arrays are built once and
    reused for every eye of every frame. invalidate() drops them.
    '''

    def __init__(self, enabled=True, margin=SWAY_MARGIN):
        self.enabled = enabled
        self.margin = margin

        self._chunks = []
        self._mins = None
        self._maxes = None

    def invalidate(self):
        self._chunks = []
        self._mins = None
        self._maxes = None

    def _refresh(self, chunks):
        self._chunks = list(chunks)

        if self._chunks:
            bounds = [chunk.world_bounds for chunk in self._chunks]
            self._mins = np.array([b[0] for b in bounds], dtype=np.float64).reshape(-1, 3)
            self._maxes = np.array([b[1] for b in bounds], dtype=np.float64).reshape(-1, 3)
        else:
            self._mins = np.zeros((0, 3))
            self._maxes = np.zeros((0, 3))

    def select(self, chunks, camera):
        '''Return the chunks to submit for `camera`.

        Falls back to submitting everything whenever culling cannot be trusted --
        disabled, or no matrix yet on the first frame -- so a missing camera
        degrades to the old behaviour rather than an empty world.
        '''
        matrix = getattr(camera, 'prepped_matrix', None)
        if (not self.enabled) or (matrix is None):
            return list(chunks)

        chunks = list(chunks)
        if (self._mins is None) or (len(self._chunks) != len(chunks)):
            self._refresh(chunks)

        mask = visible_mask(frustum_planes(matrix), self._mins, self._maxes, margin=self.margin)

        return [chunk for chunk, visible in zip(self._chunks, mask) if visible]
