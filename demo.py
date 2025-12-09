#! /usr/bin/env python3

import time
import math

from mgllib.pgwin import PygameWindow
from mgllib.mgl import MGL
from mgllib.elements import ElementSingleton
from mgllib.model.obj import OBJ
from mgllib.camera import Camera

class Demo(ElementSingleton):
    def __init__(self):
        super().__init__()

        self.window = PygameWindow(self, (1200, 1200))

    def init_mgl(self):
        self.mgl = MGL()

        self.knife_obj = OBJ("data/models/knife/knife.obj", self.mgl.program("data/shaders/default.vert", "data/shaders/default.frag"), centered=True)

        self.m4_obj = OBJ("data/models/m4/m4.obj", self.mgl.program("data/shaders/default.vert", "data/shaders/default.frag"), centered=True)

        self.watch_obj = OBJ("data/models/watch/watch.obj", self.mgl.program("data/shaders/default.vert", "data/shaders/default.frag"), centered=True)

        self.test_entity = self.knife_obj.new_entity()
        self.test_entity_2 = self.m4_obj.new_entity()
        self.test_entity_3 = self.watch_obj.new_entity()

        self.camera = Camera(up=(0, -1, 0))
        self.camera.light_pos = [0.5, 1, 1]

    def run(self):
        self.window.run()

    def update(self):
        self.mgl.ctx.clear(0.5, 0.5, 0.5)

        self.test_entity.render(self.camera)
        self.test_entity.transform.rotation[1] -= self.window.dt * 0.5

        self.test_entity_2.render(self.camera)
        self.test_entity_2.transform.rotation[1] -= self.window.dt * 1.37
        self.test_entity_2.transform.pos[0] = 2.5
        self.test_entity_2.transform.scale = [0.5, 0.5, 0.5]

        self.test_entity_3.render(self.camera)
        self.test_entity_3.transform.pos[0] = -2.5
        self.test_entity_3.transform.scale = [0.3, 0.3, 0.3]

        self.camera.set_pos((math.cos(time.time() * 0.1) * 2, math.cos(time.time() * 0.37) + 1, math.sin(time.time() * 0.1) * 2))

Demo().run()
