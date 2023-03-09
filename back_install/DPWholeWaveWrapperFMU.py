from pythonfmu import Fmi2Causality, Fmi2Variability, Fmi2Slave, Real, Integer
import agx
import agxSDK
import agxCollide
import agxRender
import agxModel
import math
import agxWire
import agxOSG
import agxPython

from sfiWindmill import SFIWindmill
from sfiWaves import WaterWrapper
from sfiSpar import SFISpar
from sfiCatamaran import SFICatamaran


class WholeWaveWrapperFMU(Fmi2Slave):
    author = "Shuai Yuan"
    description = "WholeWaveWrapperFMU"

    # this case shows the CoSim between spar and vessel.

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # Outputs
        # Velocity of the vessel
        self.register_variable(Real("Xcv", getter=lambda: self.attachH.getVelocity().x(), causality=Fmi2Causality.output))
        self.register_variable(Real("Ycv", getter=lambda: self.attachH.getVelocity().y(), causality=Fmi2Causality.output))
        self.register_variable(Real("Zcv", getter=lambda: self.attachH.getVelocity().z(), causality=Fmi2Causality.output))
        self.register_variable(
            Real("RXcv", getter=lambda: self.attachH.getAngularVelocity().x(), causality=Fmi2Causality.output))
        self.register_variable(
            Real("RYcv", getter=lambda: self.attachH.getAngularVelocity().y(), causality=Fmi2Causality.output))
        self.register_variable(
            Real("RZcv", getter=lambda: self.attachH.getAngularVelocity().z(), causality=Fmi2Causality.output))

        # Displacement of the vessel
        self.register_variable(Real("Xc", getter=lambda: self.attachH.getPosition().x(), causality=Fmi2Causality.output))
        self.register_variable(Real("Yc", getter=lambda: self.attachH.getPosition().y(), causality=Fmi2Causality.output))
        self.register_variable(Real("Zc", getter=lambda: self.attachH.getPosition().z(), causality=Fmi2Causality.output))
        self.register_variable(
            Real("RXc", getter=lambda: self.attachH.getRotation().x(), causality=Fmi2Causality.output))
        self.register_variable(
            Real("RYc", getter=lambda: self.attachH.getRotation().y(), causality=Fmi2Causality.output))
        self.register_variable(
            Real("RZc", getter=lambda: self.attachH.getRotation().z(), causality=Fmi2Causality.output))

        self.force1 = agx.Vec3()
        self.torque1 = agx.Vec3()
        self.register_variable(Real("GlobalAppliedForceXc1", getter=lambda: (-1) * self.force1.x(),
                                    causality=Fmi2Causality.output))
        self.register_variable(Real("GlobalAppliedForceYc1", getter=lambda: (-1) * self.force1.y(),
                                    causality=Fmi2Causality.output))
        self.register_variable(Real("GlobalAppliedForceZc1", getter=lambda: (-1) * self.force1.z(),
                                    causality=Fmi2Causality.output))

        self.force2 = agx.Vec3()
        self.torque2 = agx.Vec3()
        self.register_variable(Real("GlobalAppliedForceXc2", getter=lambda: (-1) * self.force2.x(),
                                    causality=Fmi2Causality.output))
        self.register_variable(Real("GlobalAppliedForceYc2", getter=lambda: (-1) * self.force2.y(),
                                    causality=Fmi2Causality.output))
        self.register_variable(Real("GlobalAppliedForceZc2", getter=lambda: (-1) * self.force2.z(),
                                    causality=Fmi2Causality.output))

        # # parameters
        self.timeStep = 0.05
        self.register_variable(
            Real('timeStep', causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
        self.size = 300
        self.register_variable(
            Real('size', causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
        self.amplitude = 0.5
        self.register_variable(
            Real('amplitude', causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
        self.period = 8
        self.register_variable(
            Real('period', causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
        self.heading = 0
        self.register_variable(
            Real('heading', causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))

        # PID parameters
        self.KpSway = 100
        self.register_variable(
            Real('Kp', causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
        self.KiSway = 0
        self.register_variable(
            Real('Kp', causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
        self.KdSway = 0
        self.register_variable(
            Real('Kp', causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))

        self.KpSurge = 100
        self.register_variable(
            Real('Kp', causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
        self.KiSurge = 0
        self.register_variable(
            Real('Kp', causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
        self.KdSurge = 0
        self.register_variable(
            Real('Kp', causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))

        self.KpHeading = 100
        self.register_variable(
            Real('Kp', causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
        self.KiHeading = 0
        self.register_variable(
            Real('Kp', causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
        self.KdHeading = 0
        self.register_variable(
            Real('Kp', causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))

        self.start_time = 1
        self.register_variable(
            Real('start_time', causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))

    def setup_experiment(self, start_time: float):
        self.init = agx.AutoInit()
        self.sim = agxSDK.Simulation()
        # agx.setNumThreads(8)

    def enter_initialization_mode(self):
        self.Sway_integral = 0
        self.Surge_integral = 0
        self.Heading_integral = 0

        self.prev_Sway_err = 0
        self.prev_Surge_err = 0
        self.prev_Heading_err = 0

        water_size = 1200
        water_depth = 135

        # floor
        floor_geometry = agxCollide.Geometry(agxCollide.Box(water_size * 0.5, water_size * 0.5, 0.5), agx.AffineMatrix4x4.translate(0, 0, 0.25))
        floor = agx.RigidBody(floor_geometry)
        floor.setMotionControl(agx.RigidBody.STATIC)
        floor.setLocalPosition(0, 0, -water_depth)
        self.sim.add(floor)
        # floor

        # water
        water_material = agx.Material("water_material")
        water_material.getBulkMaterial().setDensity(1025)

        self.water_geometry = agxCollide.Geometry(agxCollide.Box(1200, 1200, 135))
        self.water_geometry.setLocalPosition(0, 0, -135)
        self.water_geometry.setMaterial(water_material)

        self.controller = agxModel.WindAndWaterController()
        self.controller.addWater(self.water_geometry)

        self.sim.add(self.water_geometry)
        self.sim.add(self.controller)
        # water

        # wind turbine
        owt = SFIWindmill()
        owt.setLocalPosition(0, 0, 25)
        owt.setLocalRotation(agx.Quat.rotate(math.radians(90), 0, 0, 1))
        self.sim.add(owt)
        # wind turbine

        # vessel
        self.attachHmaran = SFICatamaran()
        self.attachHmaran.setLocalPosition(-9, 0, -8)
        self.sim.add(self.attachHmaran)
        self.sim.add(self.attachHmaran.mergedBody)
        # vessel

        # spar
        spar = SFISpar()
        spar.setLocalPosition(0, 0, 20)
        mooring = spar.add_mooring(floor)
        self.sim.add(spar)
        self.sim.add(mooring)
        # spar

        # create wires between spar and vessel
        # self.attachC = self.attachHmaran.getRigidBody("wireAttachment")
        self.attachH = self.attachHmaran.hull

        spar.lower_geometry.setEnableCollisions(self.attachH.geometry, False)

        deltaPosition = agx.Vec3(0, 0, 0)

        sparCon1 = agx.Vec3(3.59, 2.07, -15)
        cataCon1 = agx.Vec3(27.6, 2.07, 13)

        sparCon2 = agx.Vec3(3.59, -2.07, -15)
        cataCon2 = agx.Vec3(27.6, -2.07, 13)

        f0sc1 = agx.Frame()
        f0sc1.setLocalTranslate(deltaPosition[0] + sparCon1.x(), deltaPosition[1] + sparCon1.y(),
                                deltaPosition[2] + sparCon1.z())
        f1sc1 = agx.Frame()
        f1sc1.setLocalTranslate(deltaPosition[0] + cataCon1.x(), deltaPosition[1] + cataCon1.y(),
                                deltaPosition[2] + cataCon1.z())
        self.djsc1 = agx.DistanceJoint(spar, f0sc1, self.attachH, f1sc1)

        f0sc2 = agx.Frame()
        f0sc2.setLocalTranslate(deltaPosition[0] + sparCon2.x(), deltaPosition[1] + sparCon2.y(),
                                deltaPosition[2] + sparCon2.z())
        f1sc2 = agx.Frame()
        f1sc2.setLocalTranslate(deltaPosition[0] + cataCon2.x(), deltaPosition[1] + cataCon2.y(),
                                deltaPosition[2] + cataCon2.z())
        self.djsc2 = agx.DistanceJoint(spar, f0sc2, self.attachH, f1sc2)

        # c==l/EA, l=15, EA = 1.17e9
        Compliance_sc1 = 1.28E-8
        spookDamping_sc1 = 0.015
        self.djsc1.setCompliance(Compliance_sc1)
        self.djsc1.setDamping(spookDamping_sc1)
        self.djsc1.setEnableComputeForces(True)

        Compliance_sc2 = 1.28E-8
        spookDamping_sc2 = 0.015
        self.djsc2.setCompliance(Compliance_sc2)
        self.djsc2.setDamping(spookDamping_sc2)
        self.djsc2.setEnableComputeForces(True)

        self.sim.add(self.djsc1)
        self.sim.add(self.djsc2)

        # create wires between turbine and vessel

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

        f011 = agx.Frame()
        f011.setLocalTranslate(deltaPosition[0] + fbn111.x(), deltaPosition[1] + fbn111.y(),
                               deltaPosition[2] + fbn111.z())
        f111 = agx.Frame()
        f111.setLocalTranslate(deltaPosition[0] + fbn112.x(), deltaPosition[1] + fbn112.y(),
                               deltaPosition[2] + fbn112.z())
        self.dj11 = agx.DistanceJoint(owt, f111, self.attachH, f011)

        f012 = agx.Frame()
        f012.setLocalTranslate(deltaPosition[0] + fbn121.x(), deltaPosition[1] + fbn121.y(),
                               deltaPosition[2] + fbn121.z())
        f112 = agx.Frame()
        f112.setLocalTranslate(deltaPosition[0] + fbn122.x(), deltaPosition[1] + fbn122.y(),
                               deltaPosition[2] + fbn122.z())
        self.dj12 = agx.DistanceJoint(owt, f112, self.attachH, f012)

        f013 = agx.Frame()
        f013.setLocalTranslate(deltaPosition[0] + fbn131.x(), deltaPosition[1] + fbn131.y(),
                               deltaPosition[2] + fbn131.z())
        f113 = agx.Frame()
        f113.setLocalTranslate(deltaPosition[0] + fbn132.x(), deltaPosition[1] + fbn132.y(),
                               deltaPosition[2] + fbn132.z())
        self.dj13 = agx.DistanceJoint(owt, f113, self.attachH, f013)

        f014 = agx.Frame()
        f014.setLocalTranslate(deltaPosition[0] + fbn141.x(), deltaPosition[1] + fbn141.y(),
                               deltaPosition[2] + fbn141.z())
        f114 = agx.Frame()
        f114.setLocalTranslate(deltaPosition[0] + fbn142.x(), deltaPosition[1] + fbn142.y(),
                               deltaPosition[2] + fbn142.z())
        self.dj14 = agx.DistanceJoint(owt, f114, self.attachH, f014)

        f021 = agx.Frame()
        f021.setLocalTranslate(deltaPosition[0] + fbn211.x(), deltaPosition[1] + fbn211.y(),
                               deltaPosition[2] + fbn211.z())
        f121 = agx.Frame()
        f121.setLocalTranslate(deltaPosition[0] + fbn212.x(), deltaPosition[1] + fbn212.y(),
                               deltaPosition[2] + fbn212.z())
        self.dj21 = agx.DistanceJoint(owt, f121, self.attachH, f021)

        f022 = agx.Frame()
        f022.setLocalTranslate(deltaPosition[0] + fbn221.x(), deltaPosition[1] + fbn221.y(),
                               deltaPosition[2] + fbn221.z())
        f122 = agx.Frame()
        f122.setLocalTranslate(deltaPosition[0] + fbn222.x(), deltaPosition[1] + fbn222.y(),
                               deltaPosition[2] + fbn222.z())
        self.dj22 = agx.DistanceJoint(owt, f122, self.attachH, f022)

        f023 = agx.Frame()
        f023.setLocalTranslate(deltaPosition[0] + fbn231.x(), deltaPosition[1] + fbn231.y(),
                               deltaPosition[2] + fbn231.z())
        f123 = agx.Frame()
        f123.setLocalTranslate(deltaPosition[0] + fbn232.x(), deltaPosition[1] + fbn232.y(),
                               deltaPosition[2] + fbn232.z())
        self.dj23 = agx.DistanceJoint(owt, f123, self.attachH, f023)

        f024 = agx.Frame()
        f024.setLocalTranslate(deltaPosition[0] + fbn241.x(), deltaPosition[1] + fbn241.y(),
                               deltaPosition[2] + fbn241.z())
        f124 = agx.Frame()
        f124.setLocalTranslate(deltaPosition[0] + fbn242.x(), deltaPosition[1] + fbn242.y(),
                               deltaPosition[2] + fbn242.z())
        self.dj24 = agx.DistanceJoint(owt, f124, self.attachH, f024)

        # self.kw = 150000  # stiffness of wheel model c=1/kw=l/EA, l=30.462,5.284, EA = 7.7 e8
        Compliance_11 = 3.96E-8
        # self.cw = 0  # damping of wheel model s=cw/kw=cw*c, cw=7.6979e+06/l
        spookDamping_11 = 1.00E-2

        self.dj11.setCompliance(Compliance_11)
        self.dj11.setDamping(spookDamping_11)
        self.dj11.setEnableComputeForces(True)

        Compliance_12 = 3.96E-8
        spookDamping_12 = 1.00E-2
        self.dj12.setCompliance(Compliance_12)
        self.dj12.setDamping(spookDamping_12)
        self.dj12.setEnableComputeForces(True)

        Compliance_13 = 3.96E-8
        spookDamping_13 = 1.00E-2
        self.dj13.setCompliance(Compliance_13)
        self.dj13.setDamping(spookDamping_13)
        self.dj13.setEnableComputeForces(True)

        Compliance_14 = 3.96E-8
        spookDamping_14 = 1.00E-2
        self.dj14.setCompliance(Compliance_14)
        self.dj14.setDamping(spookDamping_14)
        self.dj14.setEnableComputeForces(True)

        Compliance_21 = 6.86E-9
        spookDamping_21 = 1.00E-2
        self.dj21.setCompliance(Compliance_21)
        self.dj21.setDamping(spookDamping_21)
        self.dj21.setEnableComputeForces(True)

        Compliance_22 = 6.86E-9
        spookDamping_22 = 1.00E-2
        self.dj22.setCompliance(Compliance_22)
        self.dj22.setDamping(spookDamping_22)
        self.dj22.setEnableComputeForces(True)

        Compliance_23 = 6.86E-9
        spookDamping_23 = 1.00E-2
        self.dj23.setCompliance(Compliance_23)
        self.dj23.setDamping(spookDamping_23)
        self.dj23.setEnableComputeForces(True)

        Compliance_24 = 6.86E-9
        spookDamping_24 = 1.00E-2
        self.dj24.setCompliance(Compliance_24)
        self.dj24.setDamping(spookDamping_24)
        self.dj24.setEnableComputeForces(True)

        self.sim.add(self.dj11)
        self.sim.add(self.dj12)
        self.sim.add(self.dj13)
        self.sim.add(self.dj14)
        self.sim.add(self.dj21)
        self.sim.add(self.dj22)
        self.sim.add(self.dj23)
        self.sim.add(self.dj24)

        self.sim.setTimeStep(self.timeStep)

    def do_step(self, current_time, step_size):
        I_N = round(step_size/self.timeStep)
        for II in range(I_N):
            wrapper = WaterWrapper(size=self.size, amplitude=self.amplitude, period=self.period, heading=self.heading, t=current_time)
            self.controller.setWaterWrapper(self.water_geometry, wrapper)

            self.djsc1.getLastForce(0, self.force1, self.torque1)
            self.djsc2.getLastForce(0, self.force2, self.torque2)

            if current_time >= self.start_time:
                currentPositionSurge = self.attachH.getPosition().x()
                currentPositionSway = self.attachH.getPosition().y()
                currentPositionHeading = self.attachH.getLocalRotation().z()

                Sway_err = 0 - currentPositionSway
                Surge_err = 0 - currentPositionSurge
                Heading_err = 0 - currentPositionHeading

                if round(current_time, 2) == self.start_time:
                    self.prev_Sway_err = Sway_err
                    self.prev_Surge_err = Surge_err
                    self.prev_Heading_err = Heading_err

                # For the proportional part
                PSway = Sway_err * self.KpSway
                PSurge = Surge_err * self.KpSurge
                PHeading = Heading_err * self.KpHeading

                # For the integral part
                # self.Sway_integral += Sway_err * dt
                self.Sway_integral += (Sway_err + self.prev_Sway_err) / 2.0 * step_size
                self.Surge_integral += (Surge_err + self.prev_Surge_err) / 2.0 * step_size
                self.Heading_integral += (Heading_err + self.prev_Heading_err) / 2.0 * step_size

                ISway = self.Sway_integral * self.KiSway
                ISurge = self.Surge_integral * self.KiSurge
                IHeading = self.Heading_integral * self.KiHeading

                # For the derivative part
                DSway_err = (Sway_err - self.prev_Sway_err) / step_size
                DSurge_err = (Surge_err - self.prev_Surge_err) / step_size
                DHeading_err = (Heading_err - self.prev_Heading_err) / step_size

                DSway = DSway_err * self.KdSway
                DSurge = DSurge_err * self.KdSurge
                DHeading = DHeading_err * self.KdHeading

                # Update Forces
                Sway_force = PSway + ISway + DSway
                Surge_force = PSurge + ISurge + DSurge
                Heading_moment = PHeading + IHeading + DHeading

                self.attachH.addForceAtLocalPosition(agx.Vec3(Sway_force, Surge_force, 0), agx.Vec3(0, 0, 0))

                self.attachH.addLocalTorque(agx.Vec3(0, 0, Heading_moment))

                self.prev_Sway_err = Sway_err
                self.prev_Surge_err = Surge_err
                self.prev_Heading_err = Heading_err

            self.sim.stepForward()

        return True


# if __name__ == '__main__':
#     kwargs = {'instance_name': '3'}
#     model = TurbineAGXW(**kwargs)
#     print(model.do_step(0.1, 0.1))
#     print(model.do_step(0.2, 0.1))
