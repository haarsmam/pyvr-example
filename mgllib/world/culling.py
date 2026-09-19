'''
Chunk visibility selection.

This is the seam where the decision "which chunks get submitted to the GPU"
lives. Today it makes no decision at all: every chunk in the world is submitted
for every eye, every frame, including chunks directly behind the head.

Keeping it as a standalone function means the current behaviour can be pinned by
a test without standing up a GL context, and means the frustum test can be
dropped in here without touching the render path.
'''


def select_visible(chunks, camera):
    '''Return the chunks to submit for `camera`.

    Currently a pass-through -- nothing is culled.
    '''
    return list(chunks)
