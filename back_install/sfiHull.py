
# import sfi
import agx
import agxOSG
import agxPython
import agxCollide
from agxUtil import createTrimeshFromFile


class SFIHull(agx.RigidBody):

    def __init__(self, visual=True):

        self.geometry = agxCollide.Geometry(createTrimeshFromFile("./Catamaran_modified6_textured.obj",
                                                                  agxCollide.Trimesh.REMOVE_DUPLICATE_VERTICES,
                                                                  agx.Matrix3x3()))

        if visual:
            g = agxOSG.GeometryNode(self.geometry)
            # n = agxOSG.readNodeFile("./Catamaran_modified6_textured.obj", False)
            # g.addChild(n)
            # root = agxPython.getContext().environment.getSceneRoot()
            # root.addChild(g)

        super().__init__(self.geometry)
        self.getMassProperties().setMass(16.803e6)
        # self.setLinearVelocityDamping(0.1)
        # self.setAngularVelocityDamping(0.1)
        InertiaTensor_diagonal = agx.SPDMatrix3x3(3.72e10, -2.56e5, 1.49e9, -2.56e5, 5.59e10, -26500, 1.49e9, -26500, 2.99e10)
        self.getMassProperties().setInertiaTensor(InertiaTensor_diagonal)

        # self.setMotionControl(agx.RigidBody.STATIC)
    def move_cm_local_translate(self, v):
        self.setCmLocalTranslate(v)

