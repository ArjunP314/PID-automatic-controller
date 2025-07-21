from copy import deepcopy
import numpy as np
import matplotlib.pyplot as plt
from Simulations import Simulator
from PIDController import PIDController
from Utils import clamp
from typing import Optional, Tuple

def mse_cost(errors, times):
    return np.mean(np.square(errors))

def iae_cost(errors, times):
    return np.mean(np.abs(errors))

def itae_cost(errors, times):
    return np.mean(np.abs(errors) * times)

def _get_cost(controller: PIDController, num_pts: int, dt: float, num_runs: int = 1, cost_func=mse_cost) -> float:
    """Return the cost of the PIDController, averaged over multiple runs if specified, using the provided cost function.
    Arguments:
        controller {PIDController} -- The PIDController to use.
        num_pts {int} -- The number of points to use for the PIDController.
        dt {float} -- The time step for the PIDController.
        num_runs {int} -- Number of simulation runs to average over (for noise robustness).
        cost_func -- Function(errors, times) -> float, cost function to use.
    Returns:
        float: The cost averaged over runs.
    """
    total_cost = 0.0
    times = np.arange(num_pts) * dt
    for _ in range(num_runs):
        c = deepcopy(controller)
        errors = np.zeros(num_pts)
        for i in range(num_pts):
            c.step(dt)
            errors[i] = c.get_error()
        total_cost += cost_func(errors, times)
    return total_cost / num_runs

class PIDTuner:
    def __init__(self, sim: Simulator, setpoint: float, t0: float, t1: float, dt: float,
                 kp_init: float = 1.0, ki_init: float = 1.0, kd_init: float = 1.0,
                 output_limits: Optional[Tuple[float, float]] = None,
                 cost_func=mse_cost):
        """Constructs a PIDTuner.
        Arguments:
            sim {Simulator} -- The Simulator to use for the PIDTuner.
            setpoint {float} -- The setpoint for the PIDTuner.
            t0 {float} -- the initial time to start the simulation at.
            t1 {float} -- the final time to end the simulation at.
            dt {float} -- the incremental time step in each simulation.
            kp_init, ki_init, kd_init {float} -- Initial PID values.
            output_limits {Optional[Tuple[float, float]]} -- Optional output clamping.
            cost_func -- Function(errors, times) -> float, cost function to use (default: MSE).
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
        self._cost_func = cost_func
        self._prev_grad = (0.0, 0.0, 0.0)
        self._prev_vals = (kp_init, ki_init, kd_init)
        self._last_mse = None

    def _get_gradient(self, delta: float = 0.01, num_runs: int = 1) -> Tuple[float, float, float]:
        """Returns the gradient of the mean squared error of the Simulations with respect to each kp, ki, and kd value using central difference, averaged over multiple runs.
        Arguments:
            delta {float} -- The small change in each kp, ki, and kd value.
            num_runs {int} -- Number of simulation runs to average over (for noise robustness).
        Returns:
            (float, float, float) -- The gradient of the mean squared error of the Simulator with respect to each kp, ki, and kd value.
        """
        kp = self._controller.kp
        ki = self._controller.ki
        kd = self._controller.kd
        # Kp
        dp_controller_plus = PIDController(deepcopy(self._sim), self._setpoint, kp + delta, ki, kd)
        dp_controller_minus = PIDController(deepcopy(self._sim), self._setpoint, kp - delta, ki, kd)
        dp = (_get_cost(dp_controller_plus, self._num_pts, self._dt, num_runs, self._cost_func) -
              _get_cost(dp_controller_minus, self._num_pts, self._dt, num_runs, self._cost_func)) / (2 * delta)
        # Ki
        di_controller_plus = PIDController(deepcopy(self._sim), self._setpoint, kp, ki + delta, kd)
        di_controller_minus = PIDController(deepcopy(self._sim), self._setpoint, kp, ki - delta, kd)
        di = (_get_cost(di_controller_plus, self._num_pts, self._dt, num_runs, self._cost_func) -
              _get_cost(di_controller_minus, self._num_pts, self._dt, num_runs, self._cost_func)) / (2 * delta)
        # Kd
        dd_controller_plus = PIDController(deepcopy(self._sim), self._setpoint, kp, ki, kd + delta)
        dd_controller_minus = PIDController(deepcopy(self._sim), self._setpoint, kp, ki, kd - delta)
        dd = (_get_cost(dd_controller_plus, self._num_pts, self._dt, num_runs, self._cost_func) -
              _get_cost(dd_controller_minus, self._num_pts, self._dt, num_runs, self._cost_func)) / (2 * delta)
        return (dp, di, dd)

    def epoch(self, gamma: float = 0.01, delta: float = 0.01, clamp_params: Tuple[float, float] = (0.0, 1e6), num_runs: int = 1) -> float:
        """Takes one step in tuning the kp, ki, and kd values.
        Arguments:
            gamma {float} -- Learning rate.
            delta {float} -- Finite difference step for gradient.
            clamp_params {Tuple[float, float]} -- Min/max for PID parameters.
            num_runs {int} -- Number of simulation runs to average over (for noise robustness).
        Returns:
            float -- The new MSE after the update.
        """
        old_vals = (self._controller.kp, self._controller.ki, self._controller.kd)
        grad = self._get_gradient(delta, num_runs=num_runs)
        new_vals = [clamp(p - gamma * g, clamp_params[0], clamp_params[1]) for p, g in zip(old_vals, grad)]
        self._controller.kp, self._controller.ki, self._controller.kd = new_vals
        cost = _get_cost(deepcopy(self._controller), self._num_pts, self._dt, num_runs, self._cost_func)
        self._last_mse = cost
        self._prev_grad = grad
        self._prev_vals = tuple(new_vals)
        return cost

    def optimize(self, max_epochs: int = 1000, tol: float = 1e-6, gamma: float = 0.01, delta: float = 0.01, clamp_params: Tuple[float, float] = (0.0, 1e6), adaptive_lr: bool = False, verbose: bool = False, num_runs: int = 1) -> Tuple[float, float, float]:
        """Run optimization until convergence or max_epochs.
        Arguments:
            max_epochs {int} -- Maximum number of epochs.
            tol {float} -- Tolerance for early stopping (gradient norm or MSE improvement).
            gamma {float} -- Learning rate.
            delta {float} -- Finite difference step for gradient.
            clamp_params {Tuple[float, float]} -- Min/max for PID parameters.
            adaptive_lr {bool} -- If True, reduce gamma if no improvement.
            verbose {bool} -- If True, print progress.
            num_runs {int} -- Number of simulation runs to average over (for noise robustness).
        Returns:
            Tuple[float, float, float] -- The optimized kp, ki, kd values.
        """
        prev_cost = float('inf')
        for epoch in range(max_epochs):
            cost = self.epoch(gamma=gamma, delta=delta, clamp_params=clamp_params, num_runs=num_runs)
            grad_norm = np.linalg.norm(self._prev_grad)
            if verbose:
                print(f"Epoch {epoch}: Cost={cost:.6f}, grad_norm={grad_norm:.6f}, kp={self._controller.kp:.4f}, ki={self._controller.ki:.4f}, kd={self._controller.kd:.4f}")
            if abs(prev_cost - cost) < tol or grad_norm < tol:
                if verbose:
                    print("Converged.")
                break
            if adaptive_lr and cost > prev_cost:
                gamma *= 0.5  # Reduce learning rate if not improving
            prev_cost = cost
        return self.get_vals()

    def multi_start_optimize(self, n_starts: int = 5, init_ranges: Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]] = ((0.1, 10.0), (0.0, 5.0), (0.0, 5.0)), **kwargs) -> Tuple[float, float, float]:
        """Run optimization from multiple initial PID values and return the best result.
        Arguments:
            n_starts {int} -- Number of random initializations.
            init_ranges {Tuple} -- Ranges for (kp, ki, kd) as ((min, max), ...).
            kwargs -- Passed to optimize().
        Returns:
            Tuple[float, float, float] -- The best kp, ki, kd values found.
        """
        best_cost = float('inf')
        best_vals = None
        for _ in range(n_starts):
            kp0 = np.random.uniform(*init_ranges[0])
            ki0 = np.random.uniform(*init_ranges[1])
            kd0 = np.random.uniform(*init_ranges[2])
            # Re-initialize controller
            self._controller = PIDController(self._sim, self._setpoint, kp0, ki0, kd0)
            vals = self.optimize(**kwargs)
            final_cost = _get_cost(deepcopy(self._controller), self._num_pts, self._dt, kwargs.get('num_runs', 1), self._cost_func)
            if final_cost < best_cost:
                best_cost = final_cost
                best_vals = vals
        # Set controller to best found
        if best_vals is not None:
            self._controller.kp, self._controller.ki, self._controller.kd = best_vals
        return best_vals

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
