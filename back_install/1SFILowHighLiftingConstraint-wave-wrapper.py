import agx
import agxSDK
import agxCollide
import math
import agxOSG
import agxPython
import agxModel
import agxIO
import sys
from agxRender import Color

from sfiSpar import SFISpar
from sfiCatamaran import SFICatamaran
from sfiWindmill import SFIWindmill
from sfiWavesS import WaterWrapper
import pandas as pd


# set up scenario
def add_floor(size, depth):
    floor_geometry = agxCollide.Geometry(
        agxCollide.Box(size * 0.5, size * 0.5, 0.5), agx.AffineMatrix4x4.translate(0, 0, 0.25))
    floor = agx.RigidBody(floor_geometry)
    floor.setMotionControl(agx.RigidBody.STATIC)
    floor.setLocalPosition(0, 0, -depth)
    return floor


# def add_water(size, depth) -> SFIWindAndWater2:
#     water_size = agx.Vec3(size, size, depth)
#     water = SFIWindAndWater2(water_size)
#     return water


def add_spar(floor):
    spar = SFISpar()
    spar.setLocalPosition(0, 0, 20)
    mooring = spar.add_mooring(floor)
    return spar, mooring


def add_catamaran() -> SFICatamaran:
    catamaran = SFICatamaran()
    catamaran.setLocalPosition(-9, 0, -8)
    # catamaran.add_static_OWTs()
    # catamaran.add_cranes()
    return catamaran


def add_lift_owt() -> SFIWindmill:
    owt = SFIWindmill()
    owt.setLocalPosition(0, 0, 25)
    # 5 meters higher than design
    owt.setLocalRotation(agx.Quat.rotate(math.radians(90), 0, 0, 1))
    return owt


def create_sky(app):
    c1 = Color.SkyBlue()
    c2 = Color.DodgerBlue()

    def to_vec3(o: Color):
        return agx.Vec3(o.x(), o.y(), o.z())

    app.getSceneDecorator().setBackgroundColor(to_vec3(c1), to_vec3(c2))


def init_camera(app, eye=agx.Vec3(-25, -25, 25), center=agx.Vec3(), up=agx.Vec3.Z_AXIS()):
    app.setCameraHome(eye, center, up)


# record position variable through time
class TimeStepEvent(agxSDK.StepEventListener):

    def __init__(self, object, filename):
        super().__init__(agxSDK.StepEventListener.PRE_STEP)
        self.object = object
        self.data = []
        self.file = filename

    def pre(self, time):
        self.data.append([time, self.object.getPosition().x(), self.object.getPosition().y(), self.object.getPosition().z(), self.object.getLocalRotation().x(), self.object.getLocalRotation().y(), self.object.getLocalRotation().z()])
        if round(time, 2) == 50.00:
            df = pd.DataFrame(self.data, columns=['time', 'x', 'y', 'z', 'Rx', 'Ry','Rz'])
            df.to_csv('./results/{}.csv'.format(self.file), index=False)


# record force variable through time
class ForceListener(agxSDK.StepEventListener):
    def __init__(self, constraints, filename):
        # We give our listener some member attributes
        super().__init__()
        self.constraints = constraints
        self.force = agx.Vec3()
        self.torque = agx.Vec3()
        self.data = []
        self.file = filename

    def pre(self, time):
        self.data.append([time, self.force.x(), self.force.y(), self.force.z()])
        if round(time, 2) == 50.00:
            df = pd.DataFrame(self.data, columns=['time', 'x', 'y', 'z'])
            df.to_csv('./results/{}.csv'.format(self.file), index=False)


# Exert force to control
class TreDPController(agxSDK.StepEventListener):

    def __init__(self, object):
        super().__init__(agxSDK.StepEventListener.PRE_STEP)

        # Controller specific members
        self.object = object
        self.TargetSway = 0
        self.TargetSurge = 0
        self.TargetHeading = 0
        self.start_time = 2.00
        self.last_time_stamp = self.start_time
        self.Sway_integral = 0
        self.Surge_integral = 0
        self.Heading_integral = 0

        self.prev_Sway_err = 0
        self.prev_Surge_err = 0
        self.prev_Heading_err = 0

        # PID parameters
        self.KpSway = 100
        self.KiSway = 0
        self.KdSway = 0

        self.KpSurge = 100
        self.KiSurge = 0
        self.KdSurge = 0

        self.KpHeading = 100
        self.KiHeading = 0
        self.KdHeading = 0

        # PID controller
    def pre(self, time):
        if round(time, 2) >= self.start_time:
            currentPositionSurge = self.object.getPosition().x()
            currentPositionSway = self.object.getPosition().y()
            currentPositionHeading = self.object.getLocalRotation().z()

            Sway_err = self.TargetSway - currentPositionSway
            Surge_err = self.TargetSurge - currentPositionSurge
            Heading_err = self.TargetHeading - currentPositionHeading

            if round(time, 2) == self.start_time:
                self.prev_Sway_err = Sway_err
                self.prev_Surge_err = Surge_err
                self.prev_Heading_err = Heading_err

            # Get delta time (dt) since last loop
            dt = time - self.last_time_stamp

            # For the proportional part
            PSway = Sway_err * self.KpSway
            PSurge = Surge_err * self.KpSurge
            PHeading = Heading_err * self.KpHeading

            # For the integral part
            # self.Sway_integral += Sway_err * dt
            self.Sway_integral += (Sway_err + self.prev_Sway_err) / 2.0 * dt
            self.Surge_integral += (Surge_err + self.prev_Surge_err) / 2.0 * dt
            self.Heading_integral += (Heading_err + self.prev_Heading_err) / 2.0 * dt

            ISway = self.Sway_integral * self.KiSway
            ISurge = self.Surge_integral * self.KiSurge
            IHeading = self.Heading_integral * self.KiHeading

            # For the derivative part
            DSway_err = (Sway_err - self.prev_Sway_err) / dt
            DSurge_err = (Surge_err - self.prev_Surge_err) / dt
            DHeading_err = (Heading_err - self.prev_Heading_err) / dt

            DSway = DSway_err * self.KdSway
            DSurge = DSurge_err * self.KdSurge
            DHeading = DHeading_err * self.KdHeading

            # Update Forces
            Sway_force = PSway + ISway + DSway
            Surge_force = PSurge + ISurge + DSurge
            Heading_moment = PHeading + IHeading + DHeading

            self.object.addForceAtLocalPosition(agx.Vec3(Sway_force, Surge_force, 0), agx.Vec3(0, 0, 0))

            self.object.addLocalTorque(agx.Vec3(0, 0, Heading_moment))

            self.last_time_stamp = time

            self.prev_Sway_err = Sway_err
            self.prev_Surge_err = Surge_err
            self.prev_Heading_err = Heading_err


# calculate
def build_scene():
    # agx.setNumThreads(4)
    sim = agxPython.getContext().environment.getSimulation()
    app = agxPython.getContext().environment.getApplication()

    sim.setTimeStep(0.05)
    app.setAutoStepping(True)

    create_sky(app)

    water_size = 1200
    water_depth = 135

    floor = add_floor(water_size, water_depth)
    spar, mooring = add_spar(floor)

    # water
    water_material = agx.Material("water_material")
    water_material.getBulkMaterial().setDensity(1025)

    water_geometry = agxCollide.Geometry(agxCollide.Box(1200, 1200, 135))
    water_geometry.setLocalPosition(0, 0, -135)
    water_geometry.setMaterial(water_material)

    controller = agxModel.WindAndWaterController()
    controller.addWater(water_geometry)
    wrapper = WaterWrapper(size=300, amplitude=1, period=8, heading=0)
    # wrapper.enable_debug_render(agx.Vec2(300, 300), 100)
    controller.setWaterWrapper(water_geometry, wrapper)

    # res = 600
    # size = agx.Vec2(500, 500)
    # hf = agxCollide.HeightField(res, res, size.x(), size.y())
    # geom = agxCollide.Geometry(hf)
    # geom.setEnableCollisions(False)
    # sim.add(geom)
    # sim.add(WaveRenderer(hf))

    # water

    catamaran = add_catamaran()
    attachH = catamaran.hull

    # AA = catamaran.mergedBody.getRigidBody()
    # AA.setMotionControl(agx.RigidBody.DYNAMICS)
    owt = add_lift_owt()

    spar.lower_geometry.setEnableCollisions(attachH.geometry, False)

    sim.add(floor)
    # sim.add(water)

    sim.add(spar)
    sim.add(mooring)
    sim.add(controller)
    sim.add(water_geometry)
    sim.add(catamaran)
    sim.add(catamaran.mergedBody)
    sim.add(owt)

    # create wires
    # attachC = catamaran.getRigidBody("wireAttach")

    deltaPosition = agx.Vec3(0, 0, 0)

    fbn111 = agx.Vec3(4, 8, 61)
    fbn112 = agx.Vec3(3.5194, 2.1992, 0)

    fbn121 = agx.Vec3(14, 8, 61)
    fbn122 = agx.Vec3(3.5194, -2.1992, 0)

    fbn131 = agx.Vec3(4, -8, 61)
    fbn132 = agx.Vec3(-3.5194, 2.1992, 0)

    fbn141 = agx.Vec3(14, -8, 61)
    fbn142 = agx.Vec3(-3.5194, -2.1992, 0)

    fbn211 = agx.Vec3(4, 8, 118)
    fbn212 = agx.Vec3(3.5194, 2.1992, 85)

    fbn221 = agx.Vec3(14, 8, 118)
    fbn222 = agx.Vec3(3.5194, -2.1992, 85)

    fbn231 = agx.Vec3(4, -8, 118)
    fbn232 = agx.Vec3(-3.5194, 2.1992, 85)

    fbn241 = agx.Vec3(14, -8, 118)
    fbn242 = agx.Vec3(-3.5194, -2.1992, 85)

    sparCon1 = agx.Vec3(3.59, 2.07, -15)
    cataCon1 = agx.Vec3(27.6, 2.07, 13)

    sparCon2 = agx.Vec3(3.59, -2.07, -15)
    cataCon2 = agx.Vec3(27.6, -2.07, 13)

    # The first will keep a distance between the two bodies
    f011 = agx.Frame()
    f011.setLocalTranslate(deltaPosition[0] + fbn111.x(), deltaPosition[1] + fbn111.y(), deltaPosition[2] + fbn111.z())
    f111 = agx.Frame()
    f111.setLocalTranslate(deltaPosition[0] + fbn112.x(), deltaPosition[1] + fbn112.y(), deltaPosition[2] + fbn112.z())
    dj11 = agx.DistanceJoint(owt, f111, attachH, f011)

    f012 = agx.Frame()
    f012.setLocalTranslate(deltaPosition[0] + fbn121.x(), deltaPosition[1] + fbn121.y(), deltaPosition[2] + fbn121.z())
    f112 = agx.Frame()
    f112.setLocalTranslate(deltaPosition[0] + fbn122.x(), deltaPosition[1] + fbn122.y(), deltaPosition[2] + fbn122.z())
    dj12 = agx.DistanceJoint(owt, f112, attachH, f012)

    f013 = agx.Frame()
    f013.setLocalTranslate(deltaPosition[0] + fbn131.x(), deltaPosition[1] + fbn131.y(), deltaPosition[2] + fbn131.z())
    f113 = agx.Frame()
    f113.setLocalTranslate(deltaPosition[0] + fbn132.x(), deltaPosition[1] + fbn132.y(), deltaPosition[2] + fbn132.z())
    dj13 = agx.DistanceJoint(owt, f113, attachH, f013)

    f014 = agx.Frame()
    f014.setLocalTranslate(deltaPosition[0] + fbn141.x(), deltaPosition[1] + fbn141.y(), deltaPosition[2] + fbn141.z())
    f114 = agx.Frame()
    f114.setLocalTranslate(deltaPosition[0] + fbn142.x(), deltaPosition[1] + fbn142.y(), deltaPosition[2] + fbn142.z())
    dj14 = agx.DistanceJoint(owt, f114, attachH, f014)

    f021 = agx.Frame()
    f021.setLocalTranslate(deltaPosition[0] + fbn211.x(), deltaPosition[1] + fbn211.y(), deltaPosition[2] + fbn211.z())
    f121 = agx.Frame()
    f121.setLocalTranslate(deltaPosition[0] + fbn212.x(), deltaPosition[1] + fbn212.y(), deltaPosition[2] + fbn212.z())
    dj21 = agx.DistanceJoint(owt, f121, attachH, f021)

    f022 = agx.Frame()
    f022.setLocalTranslate(deltaPosition[0] + fbn221.x(), deltaPosition[1] + fbn221.y(), deltaPosition[2] + fbn221.z())
    f122 = agx.Frame()
    f122.setLocalTranslate(deltaPosition[0] + fbn222.x(), deltaPosition[1] + fbn222.y(), deltaPosition[2] + fbn222.z())
    dj22 = agx.DistanceJoint(owt, f122, attachH, f022)

    f023 = agx.Frame()
    f023.setLocalTranslate(deltaPosition[0] + fbn231.x(), deltaPosition[1] + fbn231.y(), deltaPosition[2] + fbn231.z())
    f123 = agx.Frame()
    f123.setLocalTranslate(deltaPosition[0] + fbn232.x(), deltaPosition[1] + fbn232.y(), deltaPosition[2] + fbn232.z())
    dj23 = agx.DistanceJoint(owt, f123, attachH, f023)

    f024 = agx.Frame()
    f024.setLocalTranslate(deltaPosition[0] + fbn241.x(), deltaPosition[1] + fbn241.y(), deltaPosition[2] + fbn241.z())
    f124 = agx.Frame()
    f124.setLocalTranslate(deltaPosition[0] + fbn242.x(), deltaPosition[1] + fbn242.y(), deltaPosition[2] + fbn242.z())
    dj24 = agx.DistanceJoint(owt, f124, attachH, f024)

    f0sc1 = agx.Frame()
    f0sc1.setLocalTranslate(deltaPosition[0] + sparCon1.x(), deltaPosition[1] + sparCon1.y(),
                            deltaPosition[2] + sparCon1.z())
    f1sc1 = agx.Frame()
    f1sc1.setLocalTranslate(deltaPosition[0] + cataCon1.x(), deltaPosition[1] + cataCon1.y(),
                            deltaPosition[2] + cataCon1.z())
    djsc1 = agx.DistanceJoint(spar, f0sc1, attachH, f1sc1)

    f0sc2 = agx.Frame()
    f0sc2.setLocalTranslate(deltaPosition[0] + sparCon2.x(), deltaPosition[1] + sparCon2.y(),
                            deltaPosition[2] + sparCon2.z())
    f1sc2 = agx.Frame()
    f1sc2.setLocalTranslate(deltaPosition[0] + cataCon2.x(), deltaPosition[1] + cataCon2.y(),
                            deltaPosition[2] + cataCon2.z())
    djsc2 = agx.DistanceJoint(spar, f0sc2, attachH, f1sc2)

    # We relax this joint a little so we can see some spring effect
    # self.kw = 150000  # stiffness of wheel model c=1/kw=l/EA, l=30.462,5.284, EA = 7.7 e8
    Compliance_11 = 3.96E-8
    # self.cw = 0  # damping of wheel model s=cw/kw=cw*c, cw=7.6979e+06/l
    spookDamping_11 = 1.00E-2

    dj11.setCompliance(Compliance_11)
    dj11.setDamping(spookDamping_11)
    dj11.setEnableComputeForces(True)

    Compliance_12 = 3.96E-8
    spookDamping_12 = 1.00E-2
    dj12.setCompliance(Compliance_12)
    dj12.setDamping(spookDamping_12)
    dj12.setEnableComputeForces(True)

    Compliance_13 = 3.96E-8
    spookDamping_13 = 1.00E-2
    dj13.setCompliance(Compliance_13)
    dj13.setDamping(spookDamping_13)
    dj13.setEnableComputeForces(True)

    Compliance_14 = 3.96E-8
    spookDamping_14 = 1.00E-2
    dj14.setCompliance(Compliance_14)
    dj14.setDamping(spookDamping_14)
    dj14.setEnableComputeForces(True)

    Compliance_21 = 6.86E-9
    spookDamping_21 = 1.00E-2
    dj21.setCompliance(Compliance_21)
    dj21.setDamping(spookDamping_21)
    dj21.setEnableComputeForces(True)

    Compliance_22 = 6.86E-9
    spookDamping_22 = 1.00E-2
    dj22.setCompliance(Compliance_22)
    dj22.setDamping(spookDamping_22)
    dj22.setEnableComputeForces(True)

    Compliance_23 = 6.86E-9
    spookDamping_23 = 1.00E-2
    dj23.setCompliance(Compliance_23)
    dj23.setDamping(spookDamping_23)
    dj23.setEnableComputeForces(True)

    Compliance_24 = 6.86E-9
    spookDamping_24 = 1.00E-2
    dj24.setCompliance(Compliance_24)
    dj24.setDamping(spookDamping_24)
    dj24.setEnableComputeForces(True)

    # c==l/EA, l=15, EA = 1.17e9
    Compliance_sc1 = 1.28E-8
    spookDamping_sc1 = 0.015
    djsc1.setCompliance(Compliance_sc1)
    djsc1.setDamping(spookDamping_sc1)
    djsc1.setEnableComputeForces(True)

    Compliance_sc2 = 1.28E-8
    spookDamping_sc2 = 0.015
    djsc2.setCompliance(Compliance_sc2)
    djsc2.setDamping(spookDamping_sc2)
    djsc2.setEnableComputeForces(True)

    dj11.getMotor1D().setEnable(True)
    dj11.getMotor1D().setSpeed(-1)
    sim.add(dj11)
    sim.add(dj12)
    sim.add(dj13)
    sim.add(dj14)
    sim.add(dj21)
    sim.add(dj22)
    sim.add(dj23)
    sim.add(dj24)

    sim.add(djsc1)
    sim.add(djsc2)

    app.setEnableDebugRenderer(True)
    app.getSceneDecorator().setEnableShadows(True)

    # from sfiExternalForce import SFIExternalForce
    # sfi.add(SFIExternalForce(spar))
    #
    # sfi.init_camera(eye=agx.Vec3(0, 50, 100))
    # plot_window = sfiPlot.PlotWindow()
    # sim.add(plot_window)

    # displacement
    # plot_graph = plot_window.add_graph("wheel cog motion - heave", units='m', new_row=True)
    # plot_graph.add_plot("Heave-cog", f=lambda t: b1.getCmPosition().z()-15, pen=(255, 0, 0))
    # plot_graph = plot_window.add_graph("chassis cog motion - heave", units='m')
    # plot_graph.add_plot("Heave-cog", f=lambda t: b2.getCmPosition().z()-20, pen=(255, 0, 0))

    # force
    # plot_graph = plot_window.add_graph("dj1 force - axial", units='N')
    # plot_graph.add_plot("Axial-cog", f=lambda t: dj1.getCurrentForce(0), pen=(255, 0, 0))
    # plot_graph = plot_window.add_graph("dj2 force - axial", units='N')
    # plot_graph.add_plot("Axial-cog", f=lambda t: dj2.getCurrentForce(0), pen=(255, 0, 0))

    attachH_event = TimeStepEvent(attachH, 'attachH')
    sim.addEventListener(attachH_event)

    spar_event = TimeStepEvent(spar, 'spar')
    sim.addEventListener(spar_event)

    owt_event = TimeStepEvent(owt, 'owt')
    sim.addEventListener(owt_event)

    djsc1_event = ForceListener(djsc1, 'djsc11')
    sim.addEventListener(djsc1_event)

    djsc2_event = ForceListener(djsc2, 'djsc22')
    sim.addEventListener(djsc2_event)

    dj11_event = ForceListener(dj11, 'dj111')
    sim.addEventListener(dj11_event)

    dj12_event = ForceListener(dj12, 'dj112')
    sim.addEventListener(dj12_event)

    attachH_DP = TreDPController(attachH)
    sim.addEventListener(attachH_DP)

    init_camera(app, eye=agx.Vec3(0, 50, 100))


def main(args):
    # Create an application with graphics etc.
    app = agxOSG.ExampleApplication()

    # Create a command line parser. sys.executable will point to python executable
    # in this case, because getArgumentName(0) needs to match the C argv[0] which
    # is the name of the program running
    argParser = agxIO.ArgumentParser([sys.executable] + args)

    app.addScene(argParser.getArgumentName(1), "build_scene", ord('1'), True)

    # Call the init method of ExampleApplication
    # It will setup the viewer, windows etc.
    if app.init(argParser):
        app.run()
    else:
        print("An error occurred while initializing ExampleApplication.")


# Entry point when this script is loaded with python
if agxPython.getContext() is None:
    init = agx.AutoInit()
    main(sys.argv)
