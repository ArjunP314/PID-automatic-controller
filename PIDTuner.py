from copy import deepcopy
import numpy as np
import matplotlib.pyplot as plt
from Simulations import Simulator
from PIDController import PIDController
from Utils import clamp
from typing import Optional, Tuple

def _get_mse(controller: PIDController, num_pts: int, dt: float) -> float:
    """Return the mean squared error of the PIDController.
    Arguments:
        controller {PIDController} -- The PIDController to use.
        num_pts {int} -- The number of points to use for the PIDController.
        dt {float} -- The time step for the PIDController.
    Returns:
        float: The mean squared error.
    """
    mse = 0.0
    for _ in range(num_pts):
        controller.step(dt)
        err = controller.get_error()
        mse += err ** 2
    mse /= num_pts
    return mse

class PIDTuner:
    def __init__(self, sim: Simulator, setpoint: float, t0: float, t1: float, dt: float,
                 kp_init: float = 1.0, ki_init: float = 1.0, kd_init: float = 1.0,
                 output_limits: Optional[Tuple[float, float]] = None):
        """Constructs a PIDTuner.
        Arguments:
            sim {Simulator} -- The Simulator to use for the PIDTuner.
            setpoint {float} -- The setpoint for the PIDTuner.
            t0 {float} -- the initial time to start the simulation at.
            t1 {float} -- the final time to end the simulation at.
            dt {float} -- the incremental time step in each simulation.
            kp_init, ki_init, kd_init {float} -- Initial PID values.
            output_limits {Optional[Tuple[float, float]]} -- Optional output clamping.
        """
        self._sim = sim
        self._setpoint = setpoint
        self._dt = dt
        self._t0 = t0
        self._t1 = t1
        self._num_pts = int((t1 - t0) / dt)
        self._ts = np.linspace(t0, t1, num=self._num_pts)
        self._controller = PIDController(sim, setpoint, kp_init, ki_init, kd_init)
        if output_limits is not None:
            self._controller.set_output_limits(*output_limits)
        self._prev_grad = (0.0, 0.0, 0.0)
        self._prev_vals = (kp_init, ki_init, kd_init)
        self._last_mse = None

    def _get_gradient(self, delta: float = 0.01) -> Tuple[float, float, float]:
        """Returns the gradient of the mean squared error of the Simulations with respect to each kp, ki, and kd value.
        Arguments:
            delta {float} -- The small change in each kp, ki, and kd value.
        Returns:
            (float, float, float) -- The gradient of the mean squared error of the Simulator with respect to each kp, ki, and kd value.
        """
        mse_before = _get_mse(deepcopy(self._controller), self._num_pts, self._dt)
        dp_controller = PIDController(deepcopy(self._sim), self._setpoint, self._controller.kp + delta, self._controller.ki, self._controller.kd)
        di_controller = PIDController(deepcopy(self._sim), self._setpoint, self._controller.kp, self._controller.ki + delta, self._controller.kd)
        dd_controller = PIDController(deepcopy(self._sim), self._setpoint, self._controller.kp, self._controller.ki, self._controller.kd + delta)
        dp = _get_mse(dp_controller, self._num_pts, self._dt) - mse_before
        di = _get_mse(di_controller, self._num_pts, self._dt) - mse_before
        dd = _get_mse(dd_controller, self._num_pts, self._dt) - mse_before
        return (dp / delta, di / delta, dd / delta)

    def epoch(self, gamma: float = 0.01, delta: float = 0.01, clamp_params: Tuple[float, float] = (0.0, 1e6)) -> float:
        """Takes one step in tuning the kp, ki, and kd values.
        Arguments:
            gamma {float} -- Learning rate.
            delta {float} -- Finite difference step for gradient.
            clamp_params {Tuple[float, float]} -- Min/max for PID parameters.
        Returns:
            float -- The new MSE after the update.
        """
        old_vals = (self._controller.kp, self._controller.ki, self._controller.kd)
        grad = self._get_gradient(delta)
        new_vals = [clamp(p - gamma * g, clamp_params[0], clamp_params[1]) for p, g in zip(old_vals, grad)]
        self._controller.kp, self._controller.ki, self._controller.kd = new_vals
        mse = _get_mse(deepcopy(self._controller), self._num_pts, self._dt)
        self._last_mse = mse
        self._prev_grad = grad
        self._prev_vals = tuple(new_vals)
        return mse

    def optimize(self, max_epochs: int = 1000, tol: float = 1e-6, gamma: float = 0.01, delta: float = 0.01, clamp_params: Tuple[float, float] = (0.0, 1e6), adaptive_lr: bool = False, verbose: bool = False) -> Tuple[float, float, float]:
        """Run optimization until convergence or max_epochs.
        Arguments:
            max_epochs {int} -- Maximum number of epochs.
            tol {float} -- Tolerance for early stopping (gradient norm or MSE improvement).
            gamma {float} -- Learning rate.
            delta {float} -- Finite difference step for gradient.
            clamp_params {Tuple[float, float]} -- Min/max for PID parameters.
            adaptive_lr {bool} -- If True, reduce gamma if no improvement.
            verbose {bool} -- If True, print progress.
        Returns:
            Tuple[float, float, float] -- The optimized kp, ki, kd values.
        """
        prev_mse = float('inf')
        for epoch in range(max_epochs):
            mse = self.epoch(gamma=gamma, delta=delta, clamp_params=clamp_params)
            grad_norm = np.linalg.norm(self._prev_grad)
            if verbose:
                print(f"Epoch {epoch}: MSE={mse:.6f}, grad_norm={grad_norm:.6f}, kp={self._controller.kp:.4f}, ki={self._controller.ki:.4f}, kd={self._controller.kd:.4f}")
            if abs(prev_mse - mse) < tol or grad_norm < tol:
                if verbose:
                    print("Converged.")
                break
            if adaptive_lr and mse > prev_mse:
                gamma *= 0.5  # Reduce learning rate if not improving
            prev_mse = mse
        return self.get_vals()

    def get_vals(self) -> Tuple[float, float, float]:
        """Returns the computed kp, ki, and kd values.
        Returns:
            (float, float, float) -- The kp, ki, and kd values.
        """
        return (self._controller.kp, self._controller.ki, self._controller.kd)

    def get_controller(self) -> PIDController:
        """Returns the tuned PIDController.
        Returns:
            PIDController -- The tuned PIDController.
        """
        return deepcopy(self._controller)

    def plot_curve(self, label: str = '', axis = None) -> None:
        """Plots the PID curve for the tuned PIDController using matplotlib. You still need to call plt.show() to see the actual curve.
        Arguments:
            label {str} -- Label for the plot.
            axis -- Optional matplotlib axis to plot on.
        """
        controller = deepcopy(self._controller)
        data = np.zeros(len(self._ts))
        for i in range(len(self._ts)):
            controller.step(self._dt)
            data[i] = controller.sim.get_output()
        if axis is None:
            plt.plot(self._ts, data, label=label)
        else:
            axis.plot(self._ts, data, label=label)
