import math
import random
import matplotlib.pyplot as plt


# ============================================================
# PID CONTROLLER
# ============================================================

class PIDController:

    def __init__(self, kp, ki, kd, output_min=-3.0, output_max=3.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.output_min = output_min
        self.output_max = output_max

        self.integral = 0.0
        self.previous_error = 0.0

    def update(self, target, actual, dt):

        error = target - actual

        # Integral
        self.integral += error * dt

        # Derivative
        derivative = (error - self.previous_error) / dt

        # PID equation
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

wheel_base = 0.50

max_acceleration = 2.0
friction = 0.25

random.seed(7)


# ============================================================
# PID PARAMETERS
# ============================================================

left_pid = PIDController(
    kp=4.0,
    ki=0.8,
    kd=0.08,
    output_min=-2.5,
    output_max=2.5
)

right_pid = PIDController(
    kp=4.0,
    ki=0.8,
    kd=0.08,
    output_min=-2.5,
    output_max=2.5
)


# ============================================================
# ROBOT INITIAL STATE
# ============================================================

left_velocity = 0.0
right_velocity = 0.0

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
# MAIN SIMULATION LOOP
# ============================================================

steps = int(simulation_time / dt)

for step in range(steps):

    t = step * dt


    # ========================================================
    # TARGET VELOCITIES
    # ========================================================

    # 0 - 4 seconds:
    # Move straight

    if t < 4.0:

        left_target = 0.8
        right_target = 0.8


    # 4 - 8 seconds:
    # Turn left
    #
    # Left wheel slower
    # Right wheel faster

    elif t < 8.0:

        left_target = 0.75
        right_target = 0.85


    # 8 - 20 seconds:
    # Move straight again

    else:

        left_target = 0.8
        right_target = 0.8


    # ========================================================
    # SENSOR MEASUREMENT
    # ========================================================

    # Simulate small sensor noise

    left_measurement = (
        left_velocity
        + random.gauss(0.0, 0.006)
    )

    right_measurement = (
        right_velocity
        + random.gauss(0.0, 0.006)
    )


    # ========================================================
    # PID CONTROL
    # ========================================================

    left_control = left_pid.update(
        left_target,
        left_measurement,
        dt
    )

    right_control = right_pid.update(
        right_target,
        right_measurement,
        dt
    )


    # ========================================================
    # ROBOT DYNAMICS
    # ========================================================

    # Add acceleration limitation

    left_acceleration = max(
        -max_acceleration,
        min(max_acceleration, left_control)
    )

    right_acceleration = max(
        -max_acceleration,
        min(max_acceleration, right_control)
    )


    # Simple friction model

    left_acceleration -= friction * left_velocity
    right_acceleration -= friction * right_velocity


    # Update wheel velocities

    left_velocity += left_acceleration * dt
    right_velocity += right_acceleration * dt


    # Prevent reverse motion in this experiment

    left_velocity = max(0.0, left_velocity)
    right_velocity = max(0.0, right_velocity)


    # ========================================================
    # DIFFERENTIAL DRIVE KINEMATICS
    # ========================================================

    # Linear velocity of robot

    linear_velocity = (
        left_velocity + right_velocity
    ) / 2.0


    # Angular velocity

    angular_velocity = (
        right_velocity - left_velocity
    ) / wheel_base


    # Update heading

    theta += angular_velocity * dt


    # Keep theta between -pi and +pi

    theta = math.atan2(
        math.sin(theta),
        math.cos(theta)
    )


    # Update position

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


    # ========================================================
    # LOG DATA
    # ========================================================

    times.append(t)

    left_targets.append(left_target)
    right_targets.append(right_target)

    left_actuals.append(left_measurement)
    right_actuals.append(right_measurement)

    x_positions.append(x)
    y_positions.append(y)

    headings.append(theta)


# ============================================================
# PERFORMANCE ANALYSIS
# ============================================================

# Calculate absolute tracking errors

left_errors = [
    abs(target - actual)
    for target, actual
    in zip(left_targets, left_actuals)
]

right_errors = [
    abs(target - actual)
    for target, actual
    in zip(right_targets, right_actuals)
]


# Average absolute error

left_average_error = (
    sum(left_errors) / len(left_errors)
)

right_average_error = (
    sum(right_errors) / len(right_errors)
)


# Steady-state error:
# last 5 seconds of simulation

steady_state_start = int(
    15.0 / dt
)

left_steady_errors = left_errors[
    steady_state_start:
]

right_steady_errors = right_errors[
    steady_state_start:
]


left_steady_state_error = (
    sum(left_steady_errors)
    / len(left_steady_errors)
)

right_steady_state_error = (
    sum(right_steady_errors)
    / len(right_steady_errors)
)


overall_steady_error = (
    left_steady_state_error
    + right_steady_state_error
) / 2.0


# ============================================================
# FINAL VALUES
# ============================================================

final_left_velocity = left_actuals[-1]
final_right_velocity = right_actuals[-1]

final_x = x_positions[-1]
final_y = y_positions[-1]

final_heading_degrees = math.degrees(theta)


# ============================================================
# TERMINAL REPORT
# ============================================================

print()
print("=" * 60)
print("             RoboPID Simulation Complete")
print("=" * 60)

print(
    f"Final left velocity  : {final_left_velocity:.2f} m/s"
)

print(
    f"Final right velocity : {final_right_velocity:.2f} m/s"
)

print(
    f"Final X position     : {final_x:.2f} m"
)

print(
    f"Final Y position     : {final_y:.2f} m"
)

print(
    f"Final heading        : {final_heading_degrees:.2f} degrees"
)

print()
print("PID PERFORMANCE ANALYSIS")
print("-" * 60)

print(
    f"Left average error   : {left_average_error:.4f} m/s"
)

print(
    f"Right average error  : {right_average_error:.4f} m/s"
)

print(
    f"Left steady error    : {left_steady_state_error:.4f} m/s"
)

print(
    f"Right steady error   : {right_steady_state_error:.4f} m/s"
)

print(
    f"Average steady error : {overall_steady_error:.4f} m/s"
)

print("=" * 60)
print("Analysis complete.")
print("=" * 60)


# ============================================================
# FIGURE 1 — WHEEL VELOCITY RESPONSE
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
    x=4.0,
    linestyle="--",
    alpha=0.5
)

plt.axvline(
    x=8.0,
    linestyle="--",
    alpha=0.5
)

plt.xlabel("Time (seconds)")
plt.ylabel("Wheel Velocity (m/s)")

plt.title(
    "RoboPID — Two-Wheel Differential Drive"
)

plt.legend()
plt.grid(True)

plt.tight_layout()


# ============================================================
# FIGURE 2 — ROBOT TRAJECTORY
# ============================================================

plt.figure(figsize=(12, 8))

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

plt.xlabel("X position (m)")
plt.ylabel("Y position (m)")

plt.title(
    "Differential Drive Robot Trajectory"
)

plt.legend()
plt.grid(True)

plt.axis("equal")

plt.tight_layout()

plt.show()
