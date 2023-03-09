
import agx
import agxSDK

import math
from math import exp

import sfi
from sfiSpar import SFISpar


class SFIExternalForce(agxSDK.StepEventListener):

    def __init__(self, spar: SFISpar):
        super().__init__(agxSDK.StepEventListener.PRE_STEP)

        self.spar = spar

        self.__force = agx.Vec3()
        self.__torque = agx.Vec3()
        self.__zero = agx.Vec3(0, 0, 0)

        self.drag_coeff = 1
        self.added_mass_coeff = 1
        self.spar_diameter = 14
        self.__A = math.pi * pow(self.spar_diameter, 2)

        self.rho = 1025
        self.wave_amplitude = 0.5
        self.wave_frequency = 1.25
        self.__k = math.pow(self.wave_frequency, 2) / agx.GRAVITY_ACCELERATION

    def pre(self, t: float):
        self.__update_force_and_torque(t)
        self.spar.addForceAtLocalCmPosition(self.__force, self.__zero)
        self.spar.addTorque(self.__torque)

    def __update_force_and_torque(self, t: float):

        m_k = self.__k
        m_A = self.__A
        m_D = self.spar_diameter
        m_h = self.spar.getPosition().z()-self.spar.height
        m_z_COG = self.spar.cog_z

        vel = self.wave_amplitude * self.wave_frequency * exp(self.wave_frequency*t)
        signal_vel = 1 if vel >= 0 else -1

        trig_arg = self.wave_frequency * t

        c21 = self.rho * (1 + self.added_mass_coeff) * m_A
        c22 = self.wave_amplitude * math.pow(self.wave_frequency, 2) * math.sin(trig_arg)
        const_diffraction = (c21*c22) / m_k

        c11 = 0.5*self.rho*self.drag_coeff*m_D
        c12 = self.wave_amplitude * self.wave_frequency * math.cos(trig_arg)
        const_drag = (c11 * signal_vel * pow(c12, 2)) / (2*m_k)

        # force
        f_diffraction = const_diffraction * (exp(m_k*m_h) - 1.0)
        f_drag = const_drag * (exp(2*m_k * m_h) - 1.0)
        f_x = f_diffraction + f_drag
        self.__force.set(f_x, 0, 0)

        # torque
        m_diffraction = const_diffraction*((1.0 / m_k)*((m_k*m_h - 1)*exp(m_k*m_h) - 1.0) - m_z_COG*(exp(m_k*m_h - 1.0)))
        m_drag = const_drag*((1.0/(2 * m_k))*((2 * m_k*m_h - 1.0)*exp(2 * m_k*m_h) - 1.0) - m_z_COG*(exp(2 * m_k*m_h) - 1.0))
        t_y = m_diffraction + m_drag
        self.__torque.set(0, t_y, 0)
