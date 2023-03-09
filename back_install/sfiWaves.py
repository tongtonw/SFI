
import agx
import agxSDK
import agxModel
import agxCollide
import agxPython

import math
import mathUtils
# from sfiWindAndWater import SFIWindAndWater

# class SFIWaves(agxSDK.StepEventListener):
#
#     def __init__(self, wind_and_water, wave_generator=None):
#         super().__init__(agxSDK.StepEventListener.PRE_STEP)
#         self.windAndWater = wind_and_water
#         self.wavesEnabled = True
#         self.waveGenerator = wave_generator
#         self.reals = None  # type: agx.RealVector
#
#     def pre(self, t):
#
#         if self.wavesEnabled:
#
#             if self.waveGenerator is None:
#                 self.waveGenerator = mathUtils.SineWaveGenerator()
#
#             hf = self.windAndWater.heightField
#             res_x = hf.getResolutionX()
#             res_y = hf.getResolutionY()
#
#             if self.reals is None:
#                 self.reals = agx.RealVector(res_x * res_y)
#             else:
#                 self.reals.clear()
#
#             for y in range(0, res_y):
#                 for x in range(0, res_x):
#                     v = hf.getVertexFromGrid(x, y)  # type: agx.Vec3
#                     h = self.waveGenerator.get_height(self.windAndWater.size.x(), v.x(), v.y(), t)
#                     self.reals.append(h)
#
#             hf.setHeights(self.reals)
#
#
# class SFIWaves2(agxSDK.StepEventListener):
#
#     def __init__(self, wind_and_water, wave_generator=None):
#         super().__init__(agxSDK.StepEventListener.PRE_STEP)
#         self.windAndWater = wind_and_water
#         self.wavesEnabled = True
#         self.waveGenerator = wave_generator if wave_generator is not None else mathUtils.SineWaveGenerator
#         self.x_range = None
#         self.y_range = None
#
#     def pre(self, t):
#
#         if self.wavesEnabled:
#
#             hf = self.windAndWater.heightField  # type: agxCollide.HeightField
#             res_x = hf.getResolutionX()
#             res_y = hf.getResolutionY()
#
#             for y in range(0, res_y):
#                 for x in range(0, res_x):
#                     v = hf.getVertexFromGrid(x, y)  # type: agx.Vec3
#                     if self.x_range[0] < v.x() < self.x_range[1]:
#                         if self.y_range[0] < v.y() < self.y_range[1]:
#                             h = self.waveGenerator.get_height(self.windAndWater.size.x(), v.x(), v.y(), t)
#                             hf.setHeight(x, y, h)


class WaterWrapper(agxModel.WaterWrapper):

    def __init__(self, size, amplitude=1, period=2, heading=0, t=0):
        super().__init__()

        self.heading = math.radians(heading)
        self.amplitude = amplitude
        self.t = t

        self.size = size
        self.heading = heading

        f = 1.0/period
        self._omega = 2 * math.pi * f

    # def enable_debug_render(self, size: agx.Vec2, res=64):
    #
    #     sim = agxPython.getContext().environment.getSimulation()
    #
    #     hf = agxCollide.HeightField(res, res, size.x(), size.y())
    #     geom = agxCollide.Geometry(hf)
    #     geom.setEnableCollisions(False)
    #     sim.add(geom)
    #
    #     that = self
    #
    #     class WaveRenderer(agxSDK.StepEventListener):
    #
    #         def __init__(self):
    #             super().__init__(agxSDK.StepEventListener.PRE_STEP)
    #             self.real_vector = agx.RealVector(hf.getResolutionX() * hf.getResolutionY())
    #
    #         def pre(self, t):
    #             self.real_vector.clear()
    #             for y in range(0, hf.getResolutionY()):
    #                 for x in range(0, hf.getResolutionX()):
    #                     v = hf.getVertexFromGrid(x, y)  # type: agx.Vec3
    #                     h = that.get_height(agx.Vec3(v.x(), v.y(), 0), t)
    #                     self.real_vector.append(-h)
    #
    #             hf.setHeights(self.real_vector)
    #     sim.addEventListener(WaveRenderer())

    def findHeightFromSurface(self, world_point: agx.Vec3, up_vector: agx.Vec3, t: float):
        return self.get_height(world_point, self.t)

    def get_height(self, world_point: agx.Vec3, t: float):

        x = world_point.x()
        y = world_point.y()

        if self.heading != 0:
            c = math.cos(self.heading)
            s = math.sin(self.heading)
            x = x * c - y * s

        phi = mathUtils.map(x, -self.size / 8, self.size / 8, -mathUtils.TWO_PHI, mathUtils.TWO_PHI)

        surface_level = self.amplitude * math.sin(self._omega * t + phi)
        return world_point.z() - surface_level

        # waveHeight = 0.5
        # wavePeriod = 8
        # waveLength = 10
        # # 2 * math.pi / ((2 * math.pi / wavePeriod) ** 2 / 9.8)
        # for i in range(0, hf.getResolutionX()):
        #     for j in range(0, hf.getResolutionY()):
        #         index = i * hf.getResolutionX() + j
        #         value = 2 * waveHeight * (0.4 * math.cos(2 * math.pi / waveLength * j - 2 * math.pi / wavePeriod * t))

# def map(x, in_min, in_max, out_min, out_max):
#     return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# TWO_PHI = math.pi * 2
