from Simulations import simulator
from Utils import clamp

class PIDController:
    """Controller for a system using a PID loop."""

    def __init__(self, sim: 'simulator.Simulator', setpoint: float, kp: float, ki: float, kd: float):
        """Constructs a PIDController

        Arguments:
            sim {Simulator} -- The Simulator the controller will use.
            setpoint {float} -- The target value the Simulator must reach.
            kp {float} -- Proportional constant in PID.
            ki {float} -- Integral constant in PID.
            kd {float} -- Derivative constant in PID.
        """
        self.sim = sim
        self.setpoint = setpoint
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._previous_error = 0.0  # Used for the derivative term
        self._integral = 0.0        # Used for the integral term
        self._output_min = None     # Optionally set these for output clamping
        self._output_max = None

    def set_output_limits(self, output_min: float, output_max: float) -> None:
        """Set output clamping limits."""
        self._output_min = output_min
        self._output_max = output_max

    def get_error(self) -> float:
        """Returns the current error (setpoint - output)."""
        return self.setpoint - self.sim.get_output()

    def _get_input(self, dt: float) -> float:
        """Computes the P, I, and D terms to get the input that should be given to the simulated system.

        Arguments:
            dt {float} -- The time interval between time steps.

        Returns:
            float -- The input that should be given to the system.
        """
        err = self.get_error()
        p = self.kp * err

        # Anti-windup: Only integrate if output is not saturated
        self._integral += err * dt
        # Optionally clamp the integral term to prevent windup
        self._integral = clamp(self._integral, -1e6, 1e6)
        i = self.ki * self._integral

        d = self.kd * ((err - self._previous_error) / dt)

        output = p + i + d
        # Clamp output if limits are set
        if self._output_min is not None and self._output_max is not None:
            output = clamp(output, self._output_min, self._output_max)
        return output

    def step(self, dt: float) -> None:
        """Takes a step in time for the PIDController.

        The P, I, and D terms are calculated to find the new input to give to the system.
        The simulated system is then updated according to the computed input.

        Arguments:
            dt {float} -- Time step for the simulation.
        """
        inpt = self._get_input(dt)
        self.sim.set_input(inpt)
        self._previous_error = self.get_error()
        self.sim.step(dt)
