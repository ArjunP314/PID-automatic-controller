# PID Auto-tuning

This library offers a way to tune your PID feedback loops automatically using gradient descent. There are 2 example simulations given (a thermostat which must make the room attain a certain temperature), and a robot wheel (which must travel a certain distance exactly, i.e. not go over or under that distance).

Below are the plots of these simulations once they have been tuned.
![example](example.png)

## How It Works

### PID Control Theory

A PID (Proportional-Integral-Derivative) controller is a feedback control system that continuously calculates an error value as the difference between a desired setpoint and a measured process variable, and applies a correction based on proportional, integral, and derivative terms.

The PID controller output is calculated as:

```
u(t) = Kp * e(t) + Ki * ∫e(t)dt + Kd * (de/dt)
```

Where:
- **Kp (Proportional)**: Responds proportionally to the current error
- **Ki (Integral)**: Accumulates past errors to eliminate steady-state error
- **Kd (Derivative)**: Predicts future errors based on the rate of change

### Automatic Tuning with Gradient Descent

This library automatically finds optimal PID parameters (Kp, Ki, Kd) using gradient descent optimization. Here's how it works:

1. **Simulation**: The system to be controlled is simulated over a time period
2. **Error Calculation**: Mean Squared Error (MSE) is calculated between the setpoint and actual output
3. **Gradient Computation**: Partial derivatives of MSE with respect to each PID parameter are computed using finite differences
4. **Parameter Update**: PID parameters are updated using gradient descent: `new_param = old_param - learning_rate * gradient`
5. **Iteration**: Steps 1-4 are repeated until convergence

### Key Components

#### PIDController (`PIDController.py`)
- Implements the standard PID control algorithm
- Maintains integral and derivative terms
- Provides a step-by-step interface for simulation

#### PIDTuner (`PIDTuner.py`)
- Core optimization engine using gradient descent
- Computes gradients using finite difference method
- Updates PID parameters iteratively
- Provides visualization capabilities

#### Simulator Interface (`Simulations/Simulator.py`)
- Abstract base class for all simulated systems
- Defines standard interface: `set_input()`, `get_output()`, `step()`
- Allows easy addition of new system types

#### Example Simulations

**Thermostat (`Simulations/Thermostat.py`)**
- Simulates room temperature control
- Models heat transfer from heater and ambient environment
- Includes realistic noise/entropy for robustness testing

**Robot Wheel (`Simulations/RobotWheel.py`)**
- Simulates a robot wheel responding to voltage input
- Models physics including friction and responsiveness
- Demonstrates position control challenges

## Usage

### Basic Example

```python
from Simulations.Thermostat import Thermostat
from PIDTuner import PIDTuner

# Create simulation and tuner
thermostat_sim = Thermostat(entropy=0.0)
tuner = PIDTuner(thermostat_sim, setpoint=25, t0=0, t1=10, dt=0.01)

# Run optimization
for epoch in range(1000):
    tuner.epoch()

# Get optimized PID values
kp, ki, kd = tuner.get_vals()
print(f"Optimized PID: Kp={kp:.3f}, Ki={ki:.3f}, Kd={kd:.3f}")
```

### Running the Examples

```bash
python main.py
```

This will:
1. Tune both thermostat and robot wheel simulations
2. Display progress and final PID values
3. Show comparison plots of system response

### Custom Simulations

To add your own system, inherit from `Simulator`:

```python
from Simulations.Simulator import Simulator

class MySystem(Simulator):
    def __init__(self):
        super().__init__(min_input=-1.0, max_input=1.0)
        # Initialize your system state
        
    def set_input(self, val):
        # Apply input to your system
        pass
        
    def get_output(self):
        # Return current system output
        return self.output
        
    def step(self, dt):
        # Update system state for one time step
        pass
```

## Algorithm Details

### Gradient Computation

The gradient is computed using finite differences:

```
∂MSE/∂Kp ≈ (MSE(Kp+Δ) - MSE(Kp)) / Δ
∂MSE/∂Ki ≈ (MSE(Ki+Δ) - MSE(Ki)) / Δ  
∂MSE/∂Kd ≈ (MSE(Kd+Δ) - MSE(Kd)) / Δ
```

### Optimization Process

1. **Initialization**: Start with reasonable PID values (Kp=1, Ki=1, Kd=1)
2. **Simulation Run**: Simulate the system with current PID values
3. **Error Calculation**: Compute MSE over the entire simulation period
4. **Gradient Estimation**: Perturb each parameter and recompute MSE
5. **Parameter Update**: Apply gradient descent update rule
6. **Convergence Check**: Repeat until parameters stabilize

### Hyperparameters

- **Learning Rate (γ)**: Controls step size in gradient descent (default: 0.01)
- **Delta (Δ)**: Small perturbation for gradient estimation (default: 0.01)
- **Number of Epochs**: Total optimization iterations (default: 1000)
- **Time Parameters**: Simulation duration and time step (t0, t1, dt)

## Advantages

- **Automatic**: No manual tuning required
- **System-Agnostic**: Works with any system implementing the Simulator interface
- **Robust**: Handles noise and non-linearities through simulation
- **Visual**: Provides real-time plotting of optimization progress
- **Educational**: Clear implementation for learning PID control concepts

## Limitations

- **Computational Cost**: Requires multiple simulation runs per optimization step
- **Local Optima**: Gradient descent may converge to local minima
- **Simulation Accuracy**: Performance depends on simulation fidelity
- **Parameter Constraints**: Currently enforces non-negative PID values

## Requirements

- Python 3.6+
- NumPy
- Matplotlib
