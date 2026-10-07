import math
import matplotlib.pyplot as plt


# ============================================================
# ROBOT PARAMETERS
# ============================================================

dt = 0.01

wheel_base = 0.50

max_linear_velocity = 0.8
max_angular_velocity = 1.0


# ============================================================
# ROBOT INITIAL STATE
# ============================================================

x = 0.0
y = 0.0
theta = 0.0


# ============================================================
# WAYPOINTS
# ============================================================

waypoints = [
    (4.0, 0.0),
    (6.0, 2.0),
    (10.0, 2.0),
]


current_waypoint = 0


# ============================================================
# POSITION / HEADING CONTROLLER
# ============================================================

K_position = 0.8
K_heading = 2.0


# ============================================================
# DATA LOGGING
# ============================================================

x_data = []
y_data = []

target_x_data = []
target_y_data = []

heading_data = []


# ============================================================
# SIMULATION
# ============================================================

for step in range(10000):

    # --------------------------------------------------------
    # Check whether all waypoints are reached
    # --------------------------------------------------------

    if current_waypoint >= len(waypoints):
        break


    target_x, target_y = waypoints[current_waypoint]


    # --------------------------------------------------------
    # Position error
    # --------------------------------------------------------

    dx = target_x - x
    dy = target_y - y

    distance = math.sqrt(
        dx * dx + dy * dy
    )


    # --------------------------------------------------------
    # Desired heading
    # --------------------------------------------------------

    desired_heading = math.atan2(
        dy,
        dx
    )


    # --------------------------------------------------------
    # Heading error
    # --------------------------------------------------------

    heading_error = (
        desired_heading - theta
    )

    # Normalize angle to [-pi, pi]

    heading_error = math.atan2(
        math.sin(heading_error),
        math.cos(heading_error)
    )


    # --------------------------------------------------------
    # Check waypoint
    # --------------------------------------------------------

    if distance < 0.10:

        current_waypoint += 1

        continue


    # --------------------------------------------------------
    # Linear velocity
    # --------------------------------------------------------

    linear_velocity = (
        K_position * distance
    )

    linear_velocity = min(
        linear_velocity,
        max_linear_velocity
    )


    # Slow down while turning sharply

    if abs(heading_error) > math.radians(45):

        linear_velocity *= 0.3


    # --------------------------------------------------------
    # Angular velocity
    # --------------------------------------------------------

    angular_velocity = (
        K_heading * heading_error
    )

    angular_velocity = max(
        -max_angular_velocity,
        min(
            max_angular_velocity,
            angular_velocity
        )
    )


    # ========================================================
    # DIFFERENTIAL DRIVE KINEMATICS
    # ========================================================

    left_velocity = (
        linear_velocity
        - angular_velocity * wheel_base / 2
    )

    right_velocity = (
        linear_velocity
        + angular_velocity * wheel_base / 2
    )


    # ========================================================
    # ROBOT VELOCITY
    # ========================================================

    actual_linear_velocity = (
        left_velocity + right_velocity
    ) / 2


    actual_angular_velocity = (
        right_velocity - left_velocity
    ) / wheel_base


    # ========================================================
    # UPDATE ROBOT POSE
    # ========================================================

    theta += (
        actual_angular_velocity * dt
    )

    theta = math.atan2(
        math.sin(theta),
        math.cos(theta)
    )


    x += (
        actual_linear_velocity
        * math.cos(theta)
        * dt
    )
    y += (
        actual_linear_velocity
        * math.sin(theta)
        * dt
    ) 
   # ========================================================
    # LOG
    # ========================================================

    x_data.append(x)
    y_data.append(y)

    heading_data.append(
        math.degrees(theta)
    )


# ============================================================
# RESULTS
# ============================================================

print()
print("=" * 55)
print("        WAYPOINT NAVIGATION COMPLETE")
print("=" * 55)

print(
    f"Final position : "
    f"({x:.2f}, {y:.2f}) m"
)

print(
    f"Final heading  : "
    f"{math.degrees(theta):.2f} degrees"
)

print(
    f"Waypoints reached : "
    f"{current_waypoint}/{len(waypoints)}"
)
print("=" * 55)


# ============================================================
# TRAJECTORY
# ============================================================

plt.figure(figsize=(10, 8))

plt.plot(
    x_data,
    y_data,
    label="Robot Path"
)

# Plot waypoints

for i, waypoint in enumerate(waypoints):

    wx, wy = waypoint

    plt.scatter(
        wx,
        wy,
        s=80
    )

    plt.text(
        wx + 0.1,
        wy + 0.1,
        f"WP{i + 1}"
    )


# Starting point

plt.scatter(
    x_data[0],
    y_data[0],
    s=100,
    label="Start"
)

# Final Position 

plt.scatter(
    x_data[-1],
    y_data[-1],
    s=100,
    label="End"
)


plt.xlabel("X position (m)")
plt.ylabel("Y position (m)")

plt.title(
    "RoboPID — Waypoint Navigation"
)

plt.axis("equal")

plt.grid(True)

plt.legend()

plt.tight_layout()

plt.show()



