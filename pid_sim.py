import math
import random
import matplotlib.pyplot as plt


# ============================================================
# PID CONTROLLER
# ============================================================

class PIDController:

    def __init__(
        self,
        kp,
        ki,
        kd,
        output_min=-3.0,
        output_max=3.0
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.output_min = output_min
        self.output_max = output_max

        self.integral = 0.0
        self.previous_error = 0.0

    def compute(self, target, actual, dt):

        error = target - actual

        # Integral term
        self.integral += error * dt

        # Derivative term
        derivative = (
            error - self.previous_error
        ) / dt

        # PID output
        output = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
        )

        # Output limiting
        output = max(
            self.output_min,
            min(self.output_max, output)
        )

        self.previous_error = error

        return output


# ============================================================
# SIMULATION PARAMETERS
# ============================================================

dt = 0.01
simulation_time = 20.0

steps = int(simulation_time / dt)

# Robot geometry
wheel_base = 0.50

# ============================================================
# MOTOR PARAMETERS
# ============================================================

# Increased from the previous value.
# This allows the motors to reach the commanded velocity.
max_acceleration = 3.0

# Motor response time constant
motor_tau = 0.20

# Wheel friction
friction = 0.05


# ============================================================
# TARGET VELOCITIES
# ============================================================

base_velocity = 0.80

left_target = base_velocity
right_target = base_velocity


# ============================================================
# PID PARAMETERS
# ============================================================

left_pid = PIDController(
    kp=4.0,
    ki=0.8,
    kd=0.08,
    output_min=-3.0,
    output_max=3.0
)

right_pid = PIDController(
    kp=4.0,
    ki=0.8,
    kd=0.08,
    output_min=-3.0,
    output_max=3.0
)


# ============================================================
# ROBOT STATE
# ============================================================

left_velocity = 0.0
right_velocity = 0.0

left_acceleration = 0.0
right_acceleration = 0.0

x = 0.0
y = 0.0
theta = 0.0


# ============================================================
# DATA LOGGING
# ============================================================

times = []

left_targets = []
right_targets = []

left_actuals = []
right_actuals = []

x_positions = []
y_positions = []
headings = []


# ============================================================
# MAIN SIMULATION
# ============================================================

for step in range(steps):

    time = step * dt

    # --------------------------------------------------------
    # TARGET COMMAND
    # --------------------------------------------------------

    # Normal forward motion
    left_target = 0.80
    right_target = 0.80

    # Temporary differential command
    # This makes the robot turn between 4 and 8 seconds.
    if 4.0 <= time < 8.0:

        left_target = 0.75
        right_target = 0.85

    # Return to straight motion
    else:

        left_target = 0.80
        right_target = 0.80


    # --------------------------------------------------------
    # PID CONTROL
    # --------------------------------------------------------

    left_control = left_pid.compute(
        left_target,
        left_velocity,
        dt
    )

    right_control = right_pid.compute(
        right_target,
        right_velocity,
        dt
    )


    # --------------------------------------------------------
    # MOTOR DYNAMICS
    # --------------------------------------------------------

    left_acceleration_command = max(
        -max_acceleration,
        min(max_acceleration, left_control)
    )

    right_acceleration_command = max(
        -max_acceleration,
        min(max_acceleration, right_control)
    )


    # Motor response / inertia
    left_acceleration += (
        left_acceleration_command
        - left_acceleration
    ) * dt / motor_tau

    right_acceleration += (
        right_acceleration_command
        - right_acceleration
    ) * dt / motor_tau


    # --------------------------------------------------------
    # FRICTION
    # --------------------------------------------------------

    left_acceleration -= (
        friction * left_velocity
    )

    right_acceleration -= (
        friction * right_velocity
    )


    # --------------------------------------------------------
    # UPDATE WHEEL VELOCITIES
    # --------------------------------------------------------

    left_velocity += (
        left_acceleration * dt
    )

    right_velocity += (
        right_acceleration * dt
    )


    # Prevent reverse motion
    left_velocity = max(
        0.0,
        left_velocity
    )

    right_velocity = max(
        0.0,
        right_velocity
    )


    # --------------------------------------------------------
    # DIFFERENTIAL DRIVE
    # --------------------------------------------------------

    linear_velocity = (
        left_velocity + right_velocity
    ) / 2.0

    angular_velocity = (
        right_velocity - left_velocity
    ) / wheel_base


    # --------------------------------------------------------
    # ROBOT POSE UPDATE
    # --------------------------------------------------------

    theta += angular_velocity * dt

    x += (
        linear_velocity
        * math.cos(theta)
        * dt
    )

    y += (
        linear_velocity
        * math.sin(theta)
        * dt
    )


    # --------------------------------------------------------
    # SENSOR NOISE
    # --------------------------------------------------------

    left_measured = (
        left_velocity
        + random.gauss(0.0, 0.006)
    )

    right_measured = (
        right_velocity
        + random.gauss(0.0, 0.006)
    )


    # --------------------------------------------------------
    # LOG DATA
    # --------------------------------------------------------

    times.append(time)

    left_targets.append(left_target)
    right_targets.append(right_target)

    left_actuals.append(left_measured)
    right_actuals.append(right_measured)

    x_positions.append(x)
    y_positions.append(y)
    headings.append(math.degrees(theta))


# ============================================================
# PERFORMANCE ANALYSIS
# ============================================================

# Ignore initial transient when calculating steady-state error.
steady_start = int(15.0 / dt)

left_errors = []
right_errors = []

for i in range(len(times)):

    left_error = abs(
        left_targets[i] - left_actuals[i]
    )

    right_error = abs(
        right_targets[i] - right_actuals[i]
    )

    left_errors.append(left_error)
    right_errors.append(right_error)


left_average_error = sum(left_errors) / len(left_errors)
right_average_error = sum(right_errors) / len(right_errors)

left_steady_error = (
    sum(left_errors[steady_start:])
    / len(left_errors[steady_start:])
)

right_steady_error = (
    sum(right_errors[steady_start:])
    / len(right_errors[steady_start:])
)

average_steady_error = (
    left_steady_error
    + right_steady_error
) / 2.0


# ============================================================
# CONSOLE RESULTS
# ============================================================

print()
print("=" * 70)
print("                 RoboPID Simulation Complete")
print("=" * 70)

print()

print(
    f"Final left velocity  : {left_velocity:.3f} m/s"
)

print(
    f"Final right velocity : {right_velocity:.3f} m/s"
)

print(
    f"Final X position     : {x:.3f} m"
)

print(
    f"Final Y position     : {y:.3f} m"
)

print(
    f"Final heading        : {math.degrees(theta):.3f} degrees"
)

print()

print("PID PERFORMANCE")
print("-" * 70)

print(
    f"Left average error   : "
    f"{left_average_error:.4f} m/s"
)

print(
    f"Right average error  : "
    f"{right_average_error:.4f} m/s"
)

print(
    f"Left steady error    : "
    f"{left_steady_error:.4f} m/s"
)

print(
    f"Right steady error   : "
    f"{right_steady_error:.4f} m/s"
)

print(
    f"Average steady error : "
    f"{average_steady_error:.4f} m/s"
)

print("=" * 70)

print("Analysis complete.")

print("=" * 70)


# ============================================================
# GRAPH 1 — WHEEL VELOCITY RESPONSE
# ============================================================

plt.figure(figsize=(14, 7))

plt.plot(
    times,
    left_targets,
    label="Left Target"
)

plt.plot(
    times,
    left_actuals,
    label="Left Actual"
)

plt.plot(
    times,
    right_targets,
    label="Right Target"
)

plt.plot(
    times,
    right_actuals,
    label="Right Actual"
)

plt.axvline(
    4.0,
    linestyle="--",
    alpha=0.5
)

plt.axvline(
    8.0,
    linestyle="--",
    alpha=0.5
)

plt.xlabel("Time (seconds)")

plt.ylabel(
    "Wheel Velocity (m/s)"
)

plt.title(
    "RoboPID — Wheel Velocity Response"
)

plt.grid(True)

plt.legend()

plt.tight_layout()


# ============================================================
# GRAPH 2 — ROBOT TRAJECTORY
# ============================================================

plt.figure(figsize=(14, 7))

plt.plot(
    x_positions,
    y_positions,
    label="Robot Path"
)

plt.scatter(
    x_positions[0],
    y_positions[0],
    label="Start"
)

plt.scatter(
    x_positions[-1],
    y_positions[-1],
    label="End"
)

plt.xlabel(
    "X position (m)"
)

plt.ylabel(
    "Y position (m)"
)

plt.title(
    "RoboPID — Differential Drive Trajectory"
)

plt.grid(True)

plt.legend()

plt.axis("equal")

plt.tight_layout()


# ============================================================
# SHOW GRAPHS
# ============================================================

plt.show()