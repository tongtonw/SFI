import agx
import agxSDK
import agxOSG
import agxCollide
import agxPython
import agxRender

# import sfi
# import math


class SFIWindmill(agx.RigidBody):

    def __init__(self, mass=1302400, cm_local_translate=agx.Vec3(-0.3, 0, 84.2), InertiaTensor_diagonal=agx.Vec3(2.28e9, 2.29e9, 2.99e7)):
        # unit:kg,m                  with respect to local
        super().__init__()

        h = [6.5, 13, 0.5]

        cyl_rotate = agx.AffineMatrix4x4.rotate(agx.Vec3.Y_AXIS(), agx.Vec3.Z_AXIS())
        g1 = agxCollide.Geometry(agxCollide.Cylinder(3.75, h[0]), cyl_rotate * agx.AffineMatrix4x4.translate(0, 0, h[0]/2))
        g2 = agxCollide.Geometry(agxCollide.Cylinder(4.2, h[1]), cyl_rotate * agx.AffineMatrix4x4.translate(0, 0, h[1]/2))
        g3 = agxCollide.Geometry(agxCollide.Cylinder(5, h[2]), cyl_rotate * agx.AffineMatrix4x4.translate(0, 0, h[2]/2))

        g = agxOSG.GeometryNode(g1)
        n = agxOSG.readNodeFile("./windmill_top.obj", False)
        g.addChild(n)
        # root = agxPython.getContext().environment.getSceneRoot()
        # root.addChild(g)

        self.add(g1)
        self.add(g2, agx.AffineMatrix4x4.translate(0, 0, h[0]))
        self.add(g3, agx.AffineMatrix4x4.translate(0, 0, h[0] + h[1]))

        self.getMassProperties().setMass(mass)
        self.setCmLocalTranslate(cm_local_translate)
        self.getMassProperties().setInertiaTensor(InertiaTensor_diagonal)


# class SFIRotatingWindmill(agx.RigidBody):
#
#     def __init__(self):
#
#         super().__init__()
#
#         dummy_geometry1 = agxCollide.Geometry(agxCollide.Sphere(0.1))
#         dummy_geometry1.setEnableCollisions(False)
#         sim = agxPython.getContext().environment.getSimulation()
#         sim.add(dummy_geometry1)
#
#         dummy_geometry2 = agxCollide.Geometry(agxCollide.Sphere(0.1))
#         dummy_geometry2.setEnableCollisions(False)
#         dummy_geometry2.setLocalPosition(agx.Vec3(0, 0, 105.5))
#         sim.add(dummy_geometry2)
#
#         g1 = agxOSG.GeometryNode(dummy_geometry1)
#         n1 = agxOSG.readNodeFile("./windmill2.obj", False)
#         g1.addChild(n1)
#         root = agxPython.getContext().environment.getSceneRoot()
#         root.addChild(g1)
#
#         g2 = agxOSG.GeometryNode(dummy_geometry2)
#         n2 = agxOSG.readNodeFile("./windmill2_rotor.obj", False)
#         g2.addChild(n2)
#         root.addChild(g2)
#
#         self.add(dummy_geometry1)
#         self.add(dummy_geometry2)
#
#         self.setMotionControl(agx.RigidBody.KINEMATICS)
#
#         class MyListener(agxSDK.StepEventListener):
#             def __init__(self):
#                 super().__init__(agxSDK.StepEventListener.PRE_STEP)
#                 self.angle = 0
#
#             def pre(self, time):
#                 self.angle += math.radians(10) * sfi.get_time_step()
#                 dummy_geometry2.setLocalRotation(agx.Quat.rotate(self.angle, 1 ,0 ,0))
#
#         sfi.add(MyListener())
