
import agx
import agxSDK
import agxOSG
import agxCollide
import agxRender

# import sfi
# import math


class SFICrane(agx.RigidBody):

    def __init__(self, mass=330334.2, cm_local_translate=agx.Vec3(0, 0, 48.55), InertiaTensor_diagonal=agx.Vec3(259543856.2, 259543856.2, 8241024.786)):
        # unit:kg,m                  with respect to local
        super().__init__()

        h = 97.1

        cyl_rotate = agx.AffineMatrix4x4.rotate(agx.Vec3.Y_AXIS(), agx.Vec3.Z_AXIS())
        g = agxCollide.Geometry(agxCollide.Cylinder(5, h), cyl_rotate * agx.AffineMatrix4x4.translate(0, 0, h/2))

        self.add(g)

        self.getMassProperties().setMass(mass)
        self.setCmLocalTranslate(cm_local_translate)
        self.getMassProperties().setInertiaTensor(InertiaTensor_diagonal)

