# import sfi
# from sfiWindmill import SFIWindmill
# import windmill on the ship
# from sfiPlatform import SFIPlatform
# import the ship
# from sficrane import SFICrane
# import the crane
from sfiHull import SFIHull

import agx
import agxSDK
import agxOSG
import agxRender
import agxCollide
import agxPython
import osg

import math


class SFICatamaran(agxSDK.Assembly):
    # assembly owt,platform,hull,prismatic
    def __init__(self):
        super().__init__()

        self.deckHeight = 16

        self.hull = SFIHull()
        self.add(self.hull)

        self.mergedBody = agx.MergedBody()

        g = agxOSG.GeometryNode(self.hull.geometry)

        # root = agxPython.getContext().environment.getSceneRoot()
        # root.addChild(g)
        self.hull.move_cm_local_translate(agx.Vec3(62.9, 0, 21.95))

    # def add_static_OWTs(self):
    #
    #     def add_OWT(transform):
    #         owt = SFIWindmill()
    #         owt.setLocalTransform(transform)
    #         self.add(owt)
    #         self.mergedBody.add(agx.MergedBodyEmptyEdgeInteraction(self.hull, owt))
    #         return owt
    #
    #     x = 80; y = 11; z = 21.5;
    #     add_OWT(agx.AffineMatrix4x4.rotate(math.radians(90), 0, 0, 1) * agx.AffineMatrix4x4.translate(x, 0, z))
    #     add_OWT(agx.AffineMatrix4x4.rotate(math.radians(90), 0, 0, 1) * agx.AffineMatrix4x4.translate(120, 0, z))
    #     # self.owt = add_OWT\
    #     #     (agx.AffineMatrix4x4.rotate(math.radians(90), 0, 0, 1) * agx.AffineMatrix4x4.translate(42.5, 0, z))
    #
    #     self.hull.move_cm_local_translate(agx.Vec3(-10, 0, 0))

    # def add_cranes(self):
    #
    #     def add_singlecrane(transform):
    #         crane = SFICrane()
    #         crane.setLocalTransform(transform)
    #         self.add(crane)
    #         self.mergedBody.add(agx.MergedBodyEmptyEdgeInteraction(self.hull, crane))
    #         return crane
    #
    #     add_singlecrane(agx.AffineMatrix4x4.rotate(0, 0, 0, 1) * agx.AffineMatrix4x4.translate(10, 20, 21.5))
    #     add_singlecrane(agx.AffineMatrix4x4.rotate(0, 0, 0, 1) * agx.AffineMatrix4x4.translate(10, -20, 21.5))
    #
    #     self.hull.move_cm_local_translate(agx.Vec3(-10, 0, 0))

    # def add_platform(self, start_x=20, part_dist=None, gripper_start=None) -> SFIPlatform:
    #     self.platform = SFIPlatform(part_dist=part_dist, gripper_start=gripper_start)
    #     self.platform.setLocalPosition(start_x, 0, self.deckHeight + 2)
    #     self.add(self.platform)
    #     self.platform.attach(self.hull)
    #     self.hull.move_cm_local_translate(agx.Vec3(5, 0, 0))
    #
    #     for cyl in self.platform.hCylinders1 + self.platform.hCylinders2:
    #         house_geometry = cyl.house.geometry  # type: agxCollide.Geometry
    #         house_geometry.setEnableCollisions(self.hull.geometry, False)
    #
    #     return self.platform
