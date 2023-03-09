from pythonfmu import Fmi2Causality, Fmi2Variability, Fmi2Slave, Real, Integer


class AGX_control(Fmi2Slave):
    author = "Shuai Yuan"
    description = "AGX_PID-controller"

    # Control object: winch payout rate
    # target: cubic velocity Z is the same with another cubic or the velocity difference is 0

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # input to this fmu
        # self.ErrorX = 0
        # self.ErrorY = 0
        self.ErrorZ = 1

        # self.register_variable(Real("ErrorX", causality=Fmi2Causality.input))
        # self.register_variable(Real("ErrorY", causality=Fmi2Causality.input))
        self.register_variable(Real("ErrorZ", causality=Fmi2Causality.input))

        # output from this fmu
        self.Winch_PayoutRate = 0
        # This winch_PayoutRate is the rate before control start time
        self.register_variable(Real("Winch_PayoutRate", causality=Fmi2Causality.output))

        # parameters
        self.Kp = 0.01
        self.register_variable(
            Real('Kp', causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
        self.Ki = 0
        self.register_variable(
            Real('Kp', causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))
        self.Kd = 0.01
        self.register_variable(
            Real('Kp', causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))

        self.start_time = 1
        self.register_variable(
            Real('start_time', causality=Fmi2Causality.parameter, variability=Fmi2Variability.tunable))

    def enter_initialization_mode(self):
        self.prev_err = self.ErrorZ
        self.integral = 0

    def do_step(self, current_time, step_size):
        if current_time >= self.start_time:
            # For the proportional part
            P_E = self.ErrorZ * self.Kp

            # For the integral part
            self.integral += (self.ErrorZ + self.prev_err) / 2.0 * step_size
            I_E = self.integral * self.Ki

            # For the derivative part
            D_err = (self.ErrorZ - self.prev_err) / step_size
            D_E = D_err * self.Kd

            # Update PayoutRate
            self.Winch_PayoutRate = P_E + I_E + D_E
            self.prev_err = self.ErrorZ

        return True


# if __name__ == '__main__':
#     kwargs = {'instance_name': '3'}
#     model = AGX_control(**kwargs)
#     for i in range(4):
#         print(model.do_step(i, i+1))
#     # print(model.do_step(2, 1))
