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
        dx * dx +
        dy * dy
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
        desired_heading -
        theta
    )

    # Normalize angle to [-pi, pi]

    heading_error = math.atan2(
        math.sin(heading_error),
        math.cos(heading_error)
    )

    # --------------------------------------------------------
    # Log target position
    # --------------------------------------------------------

    target_x_data.append(target_x)
    target_y_data.append(target_y)

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
        K_position *
        distance
    )

    linear_velocity = min(
        linear_velocity,
        max_linear_velocity
    )

    # --------------------------------------------------------
    # Slow down while turning sharply
    # --------------------------------------------------------

    if abs(heading_error) > math.radians(45):

        linear_velocity *= 0.3

    # --------------------------------------------------------
    # Angular velocity
    # --------------------------------------------------------

    angular_velocity = (
        K_heading *
        heading_error
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
        linear_velocity -
        angular_velocity *
        wheel_base / 2
    )

    right_velocity = (
        linear_velocity +
        angular_velocity *
        wheel_base / 2
    )

    # ========================================================
    # ROBOT VELOCITY
    # ========================================================

    actual_linear_velocity = (
        left_velocity +
        right_velocity
    ) / 2

    actual_angular_velocity = (
        right_velocity -
        left_velocity
    ) / wheel_base

    # ========================================================
    # UPDATE ROBOT POSE
    # ========================================================

    theta += (
        actual_angular_velocity *
        dt
    )

    # Normalize theta

    theta = math.atan2(
        math.sin(theta),
        math.cos(theta)
    )

    x += (
        actual_linear_velocity *
        math.cos(theta) *
        dt
    )

    y += (
        actual_linear_velocity *
        math.sin(theta) *
        dt
    )

    # ========================================================
    # LOG ROBOT POSITION
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


# ============================================================
# FINAL POSITION ERROR
# ============================================================

final_wp_x, final_wp_y = waypoints[-1]

final_position_error = math.sqrt(
    (x - final_wp_x) ** 2 +
    (y - final_wp_y) ** 2
)

print(
    f"Final position error : "
    f"{final_position_error:.3f} m"
)


# ============================================================
# ACTUAL PATH LENGTH
# ============================================================

actual_path_length = 0.0

for i in range(1, len(x_data)):

    dx = x_data[i] - x_data[i - 1]
    dy = y_data[i] - y_data[i - 1]

    actual_path_length += math.sqrt(
        dx ** 2 +
        dy ** 2
    )


# ============================================================
# IDEAL PATH LENGTH
# ============================================================

# Include the robot's starting position.

reference_path = [
    (0.0, 0.0)
] + waypoints


ideal_path_length = 0.0

for i in range(
    len(reference_path) - 1
):

    x1, y1 = reference_path[i]
    x2, y2 = reference_path[i + 1]

    ideal_path_length += math.sqrt(
        (x2 - x1) ** 2 +
        (y2 - y1) ** 2
    )


# ============================================================
# PATH EFFICIENCY
# ============================================================

path_efficiency = (
    ideal_path_length /
    actual_path_length
) * 100


# ============================================================
# TRACKING ERROR METRICS
# ============================================================

tracking_errors = []


for px, py in zip(
    x_data,
    y_data
):

    minimum_error = float("inf")

    # --------------------------------------------------------
    # Calculate distance to each waypoint-to-waypoint segment
    # --------------------------------------------------------

    for i in range(
        len(waypoints) - 1
    ):

        x1, y1 = waypoints[i]
        x2, y2 = waypoints[i + 1]

        dx = x2 - x1
        dy = y2 - y1

        segment_length_squared = (
            dx ** 2 +
            dy ** 2
        )

        if segment_length_squared == 0:

            error = math.sqrt(
                (px - x1) ** 2 +
                (py - y1) ** 2
            )

        else:

            t = (
                (px - x1) * dx +
                (py - y1) * dy
            ) / segment_length_squared

            t = max(
                0.0,
                min(1.0, t)
            )

            closest_x = (
                x1 +
                t * dx
            )

            closest_y = (
                y1 +
                t * dy
            )

            error = math.sqrt(
                (px - closest_x) ** 2 +
                (py - closest_y) ** 2
            )

        minimum_error = min(
            minimum_error,
            error
        )

    tracking_errors.append(
        minimum_error
    )


# ============================================================
# MAXIMUM TRACKING ERROR
# ============================================================

maximum_tracking_error = max(
    tracking_errors
)


# ============================================================
# AVERAGE TRACKING ERROR
# ============================================================

average_tracking_error = (
    sum(tracking_errors) /
    len(tracking_errors)
)


# ============================================================
# RMSE TRACKING ERROR
# ============================================================

rmse_tracking_error = math.sqrt(
    sum(
        error ** 2
        for error in tracking_errors
    ) /
    len(tracking_errors)
)


# ============================================================
# PRINT TRACKING METRICS
# ============================================================

print(
    f"Maximum tracking error : "
    f"{maximum_tracking_error:.3f} m"
)

print(
    f"Average tracking error : "
    f"{average_tracking_error:.3f} m"
)

print(
    f"RMSE tracking error    : "
    f"{rmse_tracking_error:.3f} m"
)


# ============================================================
# PRINT PATH METRICS
# ============================================================

print(
    f"Actual path length     : "
    f"{actual_path_length:.3f} m"
)

print(
    f"Ideal path length      : "
    f"{ideal_path_length:.3f} m"
)

print(
    f"Path efficiency        : "
    f"{path_efficiency:.2f} %"
)

print(
    f"Waypoints reached      : "
    f"{current_waypoint}/{len(waypoints)}"
)

print(
    "=" * 55
)


# ============================================================
# WAYPOINT NAVIGATION PLOT
# ============================================================

plt.figure(
    figsize=(12, 8)
)

# Robot path

plt.plot(
    x_data,
    y_data,
    label="Robot Path"
)

# Start point

plt.scatter(
    0,
    0,
    color="red",
    s=120,
    label="Start",
    zorder=5
)

# Waypoints

waypoint_x = [
    point[0]
    for point in waypoints
]

waypoint_y = [
    point[1]
    for point in waypoints
]

plt.scatter(
    waypoint_x,
    waypoint_y,
    color="orange",
    s=100,
    zorder=5
)


# End point

plt.scatter(
    x,
    y,
    color="purple",
    s=120,
    label="End",
    zorder=5
)


# ------------------------------------------------------------
# Waypoint labels
# ------------------------------------------------------------

for i, (wx, wy) in enumerate(
    waypoints,
    start=1
):

    plt.annotate(
        f"WP{i}",
        (wx, wy),
        xytext=(8, 8),
        textcoords="offset points",
        fontsize=11
    )


# ------------------------------------------------------------
# Plot formatting
# ------------------------------------------------------------

plt.xlabel(
    "X position (m)"
)

plt.ylabel(
    "Y position (m)"
)

plt.title(
    "RoboPID — Waypoint Navigation"
)

plt.grid(
    True,
    alpha=0.4
)

plt.legend()

plt.axis("equal")

plt.tight_layout()

plt.show()
