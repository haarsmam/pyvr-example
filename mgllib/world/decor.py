from array import array

import glm

from ..elements import Element
from ..model.vao import TexturedVAOs
from .const import BASE_DECOR_FORMAT, DECOR_FORMAT

class DecorGroup(Element):
    def __init__(self, items):
        super().__init__()

        self.items = items

        self.mgl_buffer = None

        if len(self.items):
            self.program = self.items[0].source.vao.program

            self.buffer = array('f', [])

            for item in self.items:
                self.buffer += item.buffer

            self.mgl_buffer = self.e['MGL'].ctx.buffer(data=self.buffer)

            vao = self.e['MGL'].ctx.vertex_array(self.program, [(self.mgl_buffer, *DECOR_FORMAT)])

            self.vao = TexturedVAOs(self.program, [vao])

            # get textures and layout
            self.vao.textures = items[0].source.vao.textures.copy()
            self.texture_flags = items[0].source.vao.texture_flags

    def release(self):
        if self.mgl_buffer:
            self.mgl_buffer.release()
            self.mgl_buffer = None

class Decor(Element):
    def __init__(self, source_obj, pos=glm.vec3(), rot=glm.quat(), scale=glm.vec3(1.0)):
        super().__init__()

        self.source = source_obj

        self.pos = glm.vec3(pos)
        self.rot = glm.quat(rot)
        self.scale = glm.vec3(scale)

        self.generate_buffer()

    def generate_buffer(self):
        self.transform = glm.translate(self.pos) * glm.mat4(self.rot) * glm.scale(self.scale)
        self.normal_transform = glm.transpose(glm.inverse(self.transform))

        buffer = []

        # Vertices are baked into world space here and the decor shaders ignore
        # world_transform, so these bounds are what culling has to test against.
        # A tree reaches well outside the chunk that owns its base.
        low = [float('inf')] * 3
        high = [float('-inf')] * 3

        for material in self.source.geometry.materials:
            if len(self.source.geometry.materials[material]):
                for vertex in self.source.geometry.materials[material]:
                    for group in BASE_DECOR_FORMAT:
                        item = vertex[group]
                        if group == 'vert':
                            item = tuple(self.transform * glm.vec3(item))
                            for axis in range(3):
                                low[axis] = min(low[axis], item[axis])
                                high[axis] = max(high[axis], item[axis])
                        if group == 'normal':
                            item = tuple(glm.normalize(self.normal_transform * glm.vec4(glm.vec3(item), 0.0)).xyz)
                        for v in item:
                            buffer.append(v)
                    for v in self.pos:
                        buffer.append(v)
        
        self.buffer = array('f', buffer)

        # An empty model has no extent; collapse it onto its own origin.
        if low[0] > high[0]:
            low = list(self.pos)
            high = list(self.pos)

        self.bounds = (tuple(low), tuple(high))
