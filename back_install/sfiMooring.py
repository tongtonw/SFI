import agx
import agxSDK
import agxCollide
import agxWire
import agxRender

# import sfi
import math


class SFIMooring(agxSDK.Assembly):

    def __init__(self, spar, floor):
        super().__init__()

        self.getFrame().setParent(spar.getFrame())

        material = agx.Material("wire_material")

        bulk_material = material.getBulkMaterial()  # type: agx.BulkMaterial
        bulk_material.setDensity(27910)
        # bulk_material.setDensity(27910)

        wire_material = material.getWireMaterial()  # type: agx.WireMaterial
        wire_material.setYoungsModulusStretch(64e9)
        wire_material.setYoungsModulusBend(64e9)

        self.mass = 0

        for angle in [120, 0, -120]:
            line = MooringLine(spar, floor, spar.lowerRadius + 0.1, 10, 600,
                               math.radians(angle), math.radians(15),
                               material)
            self.mass += line.mass
            self.add(line)


class MooringLine(agxSDK.Assembly):

    def __init__(self, spar, floor, radius1, radius2, radius3, angle1, angle2, mat: agx.Material):
        super().__init__()

        h = 100 - spar.getPosition().z() - 90

        t1 = agx.AffineMatrix4x4.translate(0, 0, -90)
        t2 = agx.AffineMatrix4x4.rotate(angle1, 0, 0, 1)
        t3 = agx.AffineMatrix4x4.translate(radius1, 0, 0)
        t4 = agx.AffineMatrix4x4.rotate(math.atan2(h, radius1 + radius2 + radius3), 0, 1, 0)
        t5 = agx.AffineMatrix4x4.translate(radius2, 0, 0)

        t = t5 * t4 * t3 * t2 * t1

        link_material = agx.Material("link_material")
        link_material.getBulkMaterial().setDensity(3600)

        link_geometry = agxCollide.Geometry(agxCollide.Cylinder(0.25, 1),
                                            agx.AffineMatrix4x4.rotate(agx.Vec3.Y_AXIS(), agx.Vec3.X_AXIS()))
        link_geometry.setMaterial(link_material)
        link_geometry.setEnableCollisions(False)

        link_body = agx.RigidBody(link_geometry)
        link_body.setLocalTransform(t)
        link_body.setParentFrame(self.getFrame())

        self.link = agxWire.Link(link_body)

        def create_wire1(left: bool) -> agxWire.Wire:
            sign = 1 if left else -1
            x = radius1 * math.cos(angle1 + angle2 * sign)
            y = radius1 * math.sin(angle1 + angle2 * sign)
            wire = agxWire.Wire(0.1, 3)
            wire.setEnableCollisions(False)
            wire.setMaterial(mat)
            wire.add(agxWire.BodyFixedNode(spar, x, y, -56))
            self.link.connect(wire, agx.Vec3(-0.5, 0, 0), agxWire.Link.WIRE_END)
            wire.add(self.link)
            self.add(wire)
            return wire

        def create_wire2() -> agxWire.Wire:
            r = radius1 + radius2 + radius3
            x = r * math.cos(angle1)
            y = r * math.sin(angle1)
            wire = agxWire.Wire(0.05, 3)
            #wire.setEnableCollisions(False)
            wire.setMaterial(mat)
            self.link.connect(wire, agx.Vec3(0.5, 0, 0), agxWire.Link.WIRE_BEGIN)
            wire.add(self.link)
            wire.add(agxWire.BodyFixedNode(floor, x, y, 0.5))
            self.add(wire)
            return wire

        self.left_wire = create_wire1(True)
        self.right_wire = create_wire1(False)
        self.main_wire = create_wire2()

        self.mass = self.left_wire.getMass() + self.right_wire.getMass() + self.main_wire.getMass()
       # print("Wire mass={}".format(self.mass))
