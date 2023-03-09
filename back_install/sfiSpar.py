import agx
import agxSDK
import agxCollide
import agxModel
import agxPython
from agxUtil import createTrimeshFromFile
# import sfi
from sfiMooring import SFIMooring

mass = 12.642e6
# mass = 12.642e6
# kg

custom_data = dict()


# def get_contact_force(body: agx.RigidBody):
#     sim = agxSDK.Simulation()
#     space = sim.getSpace()  # type: agxCollide.Space
#     contacts = []
#     for gc in space.getGeometryContacts():  # type: agxCollide.GeometryContact
#         if not gc.isEnabled():
#             continue
#         rb1 = gc.rigidBody(0)  # type: agx.RigidBody
#         rb2 = gc.rigidBody(1)  # type: agx.RigidBody
#         if rb1 is not None and rb1.getUuid() != body.getUuid() and rb2 is not None and rb2.getUuid() != body.getUuid():
#             continue
#         points = gc.points()  # type: agxCollide.ContactPointVector
#         for point in points:  # type: agxCollide.ContactPoint
#             if not point.getEnabled():
#                 continue
#             contacts.append(point)
#
#     return contacts


class SFISpar(agx.RigidBody):

    def __init__(self):

        super(SFISpar, self).__init__()

        self.upperRadius = 4.15  # 8.3 for diameter
        self.lowerRadius = 7.5  # 15 for diameter
        # Hong model different from B model

        h = [1, 10, 10, 70]
        self.height = sum(h)-1
        self.shapes = []

        def _top_geometry():
            shape = createTrimeshFromFile("./spar_middle_top.obj", agxCollide.Trimesh.REMOVE_DUPLICATE_VERTICES, agx.Matrix3x3())
            self.shapes.append((shape, "spar_top"))
            geometry = agxCollide.Geometry(shape)
            geometry.setEnableCollisions(False)
            return geometry

        def _upper_geometry():
            shape = agxCollide.Cylinder(self.upperRadius, h[1])
            self.shapes.append((shape, "spar_upper"))
            geometry = agxCollide.Geometry(shape, agx.AffineMatrix4x4.rotate(agx.Vec3.Y_AXIS(), agx.Vec3.Z_AXIS())
                                           * agx.AffineMatrix4x4.translate(0, 0, -h[1] / 2))
            geometry.setEnableCollisions(False)
            return geometry

        def _middle_geometry():
            shape = createTrimeshFromFile("./spar_middle.obj", agxCollide.Trimesh.REMOVE_DUPLICATE_VERTICES, agx.Matrix3x3())
            self.shapes.append((shape, "spar_middle"))
            geometry = agxCollide.Geometry(shape)
            geometry.setEnableCollisions(False)
            return geometry

        def _lower_geometry():
            shape = agxCollide.Cylinder(self.lowerRadius, h[3])
            self.shapes.append((shape, "spar_lower"))
            geometry = agxCollide.Geometry(shape,  agx.AffineMatrix4x4.rotate(agx.Vec3.Y_AXIS(), agx.Vec3.Z_AXIS())
                                           * agx.AffineMatrix4x4.translate(0, 0, -h[3] / 2))
            return geometry

        self.top_geometry = _top_geometry()
        self.upper_geometry = _upper_geometry()
        self.middle_geometry = _middle_geometry()
        self.lower_geometry = _lower_geometry()

        self.add(self.upper_geometry, agx.AffineMatrix4x4.translate(0, 0, -h[0]))
        self.add(self.middle_geometry, agx.AffineMatrix4x4.translate(0, 0, 10 - h[1]))
        self.add(self.lower_geometry, agx.AffineMatrix4x4.translate(0, 0, -h[0] - h[1] - h[2]))

        # self.setLinearVelocityDamping(0.05)
        # self.setAngularVelocityDamping(0.05)

        if 'spar_material' in custom_data:
            self.upper_geometry.setMaterial(custom_data['spar_material'])
            print('applied material to spar')
        # mass center
        self.cog_z = -62.074
        # self.cog_z = -52.074
        self.getMassProperties().setMass(mass)
        self.setCmLocalTranslate(agx.Vec3(0, 0, self.cog_z))
        InertiaTensor_diagonal = agx.Vec3(5.11e9, 5.11e9, 4.10e8)
        self.getMassProperties().setInertiaTensor(InertiaTensor_diagonal)

    def add_mooring(self, floor):
        mooring = SFIMooring(self, floor)
        spar_mass = self.getMassProperties().getMass()
        self.getMassProperties().setMass(spar_mass - mooring.mass)
        # sfi.add(SFIMooring(self, floor))
        return mooring

    def hydro_setup(self, controller: agxModel.WindAndWaterController):
        for shape in self.shapes:
            params = controller.getOrCreateHydrodynamicsParameters(shape[0])  # type: agxModel.HydrodynamicsParameters
            params.setShapeTessellationLevel(agxModel.HydrodynamicsParameters.ULTRA_HIGH)
            # params.initializeAddedMassStorage("addedmassstorage/{}.dat".format(shape[1]))


# class SFISpar2(agx.RigidBody):
#
#     def __init__(self):
#         super().__init__()
#
#         self.upperRadius = 4.5
#         self.lowerRadius = 7
#
#         h = [1, 15, 10, 70]
#         self.height = sum(h)
#
#         self.geometry = agxCollide.Geometry()
#
#         self.geometry.add(createTrimeshFromFile("./spar_middle_top.obj", agxCollide.Trimesh.REMOVE_DUPLICATE_VERTICES, agx.Matrix3x3()))
#         self.geometry.add(agxCollide.Cylinder(self.upperRadius, h[1]), agx.AffineMatrix4x4.rotate(agx.Vec3.Y_AXIS(), agx.Vec3.Z_AXIS())
#                           * agx.AffineMatrix4x4.translate(0, 0, (-h[1] / 2) - h[0]))
#
#         self.geometry.add(createTrimeshFromFile("./spar_middle.obj", agxCollide.Trimesh.REMOVE_DUPLICATE_VERTICES, agx.Matrix3x3()), agx.AffineMatrix4x4.translate(0, 0, 10 - h[1]))
#
#         self.geometry.add(agxCollide.Cylinder(7, h[3]), agx.AffineMatrix4x4.rotate(agx.Vec3.Y_AXIS(), agx.Vec3.Z_AXIS())
#                           * agx.AffineMatrix4x4.translate(0, 0 ,( -h[3] / 2) - h[0] - h[1] - h[2]))
#
#         self.add(self.geometry)
#
#         self.setLinearVelocityDamping(0.05)
#         self.setAngularVelocityDamping(0.05)
#
#         if 'spar_material' in custom_data:
#             self.geometry.setMaterial(custom_data['spar_material'])
#             print('applied material to spar')
#
#         self.cog_z = -self.height + 30
#
#         self.getMassProperties().setMass(mass)
#         self.setCmLocalTranslate(agx.Vec3(0, 0, self.cog_z))
#
#     def add_mooring(self, floor) -> SFIMooring:
#         mooring = SFIMooring(self, floor)
#         # spar_mass = self.getMassProperties().getMass()
#         # self.getMassProperties().setMass(spar_mass - mooring.mass)
#         sim = agxSDK.Simulation()
#         return sim.add(mooring)
#
#     def hydro_setup(self, controller: agxModel.WindAndWaterController):
#         for shape in self.geometry.getShapes():  # type: agxCollide.Shape
#             params = controller.getOrCreateHydrodynamicsParameters(shape)  # type: agxModel.HydrodynamicsParameters
#
#             # params.initializeAddedMassStorage("addedmassstorage/{}.dat".format(shape.getUuid()))
#             params.setShapeTessellationLevel(agxModel.HydrodynamicsParameters.ULTRA_HIGH)
#
#
# class FixedSpar(agx.RigidBody):
#
#     def __init__(self, radius, height):
#
#         self.radius = radius
#         self.height = height
#
#         self.geometry = agxCollide.Geometry(
#             agxCollide.Cylinder(radius, height), agx.AffineMatrix4x4.rotate(agx.Vec3.Y_AXIS(), agx.Vec3.Z_AXIS())
#                                       * agx.AffineMatrix4x4.translate(0, 0, -height/2))
#
#         super(FixedSpar, self).__init__(self.geometry)
#
#         self.setMotionControl(agx.RigidBody.KINEMATICS)
#
#
# class SparForceListener(agxSDK.StepEventListener):
#
#     def __init__(self, spar):
#         super().__init__(agxSDK.StepEventListener.POST_STEP)
#         self.spar = spar
#         self.has_made_contact = False
#         self.data = ["Time, normal_force_magnitude"]
#
#     def post(self, time):
#         contacts = get_contact_force(self.spar)
#         sum_normal_force_magnitude = 0
#         if len(contacts) > 0:
#             if not self.has_made_contact:
#                 self.has_made_contact = True
#             for contact in contacts:  # type: agxCollide.ContactPoint
#                 sum_normal_force_magnitude += contact.getNormalForceMagnitude()
#             sum_normal_force_magnitude /= 1e6
#             print('sum normalForceMagnitude={}'.format(sum_normal_force_magnitude))
#         self.data.append("{}, {}".format(time, sum_normal_force_magnitude))
