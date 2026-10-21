
from utils.brick import Motor, reset_brick, wait_ready_sensors, EV3ColorSensor
import time
import math

# Constants from Lab 2
RADIUS = 2.2
TRACK_WIDTH = 11.5
SPEED = -100
TILE = 30.48
THRESHOLD = 60
STEPS = 3
CORRECTION = 0

leftmotor = Motor("B")
rightmotor = Motor("C")
colorsensor = EV3ColorSensor(2)

x = 0.0
y = 0.0
theta = 0.0

leftmotor.reset_encoder()
rightmotor.reset_encoder()
prev_right_encoder = 0
prev_left_encoder = 0

wait_ready_sensors()


def update_odometer():
    # Existing odometry function from Lab 2
    global x, y, theta, prev_right_encoder, prev_left_encoder

    current_left_encoder = -leftmotor.get_encoder()
    current_right_encoder = -rightmotor.get_encoder()

    delta_left = current_left_encoder - prev_left_encoder
    delta_right = current_right_encoder - prev_right_encoder

    delta_distance_left = (
        delta_left / 360.0
    ) * 2 * math.pi * RADIUS

    delta_distance_right = (
        delta_right / 360.0
    ) * 2 * math.pi * RADIUS

    delta_theta = (
        delta_distance_left - delta_distance_right
    ) / TRACK_WIDTH

    delta_distance = (
        delta_distance_left + delta_distance_right
    ) / 2.0

    # Heading convention: 0 degrees points along +y,
    # increasing clockwise toward +x.
    delta_x = delta_distance * math.sin(
        math.radians(theta) + delta_theta / 2.0
    )
    delta_y = delta_distance * math.cos(
        math.radians(theta) + delta_theta / 2.0
    )

    x += delta_x
    y += delta_y
    theta = (theta + math.degrees(delta_theta)) % 360.0

    prev_left_encoder = current_left_encoder
    prev_right_encoder = current_right_encoder


# NEW: Check whether a signed target displacement is complete.
def target_reached(current_position, target_position, distance):
    if distance >= 0:
        return current_position >= target_position
    else:
        return current_position <= target_position


# CHANGED: Supports positive AND negative distances.
def drive_forward(distance, direction):
    if direction == "x":
        start_position = x
    else:
        start_position = y

    target_position = start_position + distance

    while True:
        current_position = x if direction == "x" else y

        if target_reached(current_position, target_position, distance):
            break

        leftmotor.set_dps(SPEED)
        rightmotor.set_dps(SPEED)

        update_odometer()

        print(
            f"X: {x:.2f} cm | Y: {y:.2f} cm | "
            f"Theta: {theta:.2f}°",
            end="\r"
        )

        time.sleep(0.01)

    leftmotor.set_dps(0)
    rightmotor.set_dps(0)


def steer(direction):
    slow = SPEED * CORRECTION

    if direction == "right":
        leftmotor.set_dps(SPEED * 0.25)
        rightmotor.set_dps(slow)
    else:
        leftmotor.set_dps(slow)
        rightmotor.set_dps(SPEED * 0.25)


def correct_path():
    for _ in range(STEPS):
        steer("right")
        update_odometer()
        time.sleep(0.01)

        if colorsensor.get_value()[0] < THRESHOLD:
            return

    for _ in range(STEPS):
        steer("left")
        update_odometer()

        if colorsensor.get_value()[0] < THRESHOLD:
            return

        time.sleep(0.01)


# CHANGED: Signed distance determines which coordinate direction
# completes the movement. Color-sensor correction is preserved.
def drive_forward_correct(distance, direction):
    if direction == "x":
        start_position = x
    else:
        start_position = y

    target_position = start_position + distance

    while True:
        current_position = x if direction == "x" else y

        if target_reached(current_position, target_position, distance):
            break

        if colorsensor.get_value()[0] < THRESHOLD:
            leftmotor.set_dps(SPEED)
            rightmotor.set_dps(SPEED)
        else:
            correct_path()

        update_odometer()

        print(
            f"X: {x:.2f} cm | Y: {y:.2f} cm | "
            f"Theta: {theta:.2f}°",
            end="\r"
        )

        time.sleep(0.01)

    leftmotor.set_dps(0)
    rightmotor.set_dps(0)


def get_shortest_angle(target_heading, current_heading):
    """Return the shortest signed turn to reach a target heading."""
    return (target_heading - current_heading + 180) % 360 - 180


def execute_turn(target_heading):
    """Turn to a target heading using the shortest direction."""
    target_heading = target_heading % 360.0
    delta = get_shortest_angle(target_heading, theta)

    # Already facing the target within 2 degrees
    if abs(delta) < 2:
        return

    # Positive delta = clockwise/right
    # Negative delta = counterclockwise/left
    if delta > 0:
        leftmotor.set_dps(SPEED)
        rightmotor.set_dps(-SPEED)
    else:
        leftmotor.set_dps(-SPEED)
        rightmotor.set_dps(SPEED)

    try:
        while abs(get_shortest_angle(target_heading, theta)) > 2:
            update_odometer()
            time.sleep(0.01)
    finally:
        leftmotor.set_dps(0)
        rightmotor.set_dps(0)


def travel_safe(waypoint):
    # CHANGED: Keep the sign of each displacement.
    # Positive dy means +y; negative dy means -y.
    dy = waypoint[1] * TILE - y

    if abs(dy) > 1:
        execute_turn(0 if dy > 0 else 180)
        drive_forward_correct(dy, "y")

    # Positive dx means +x; negative dx means -x.
    dx = waypoint[0] * TILE - x

    if abs(dx) > 1:
        execute_turn(90 if dx > 0 else 270)
        drive_forward_correct(dx, "x")

def drive_straight_distance(distance, heading):
    global x, y

    start_x = x
    start_y = y

    heading_rad = math.radians(heading)
    progress = 0.0

    try:
        while progress < distance:
            leftmotor.set_dps(SPEED)
            rightmotor.set_dps(SPEED)

            update_odometer()

            # Calculate distance travelled in the intended direction.
            progress = (
                (x - start_x) * math.sin(heading_rad)
                + (y - start_y) * math.cos(heading_rad)
            )

            print(
                f"X: {x:.2f} cm | Y: {y:.2f} cm | "
                f"Theta: {theta:.2f}° | "
                f"Distance: {progress:.2f}/{distance:.2f} cm",
                end="\r"
            )

            time.sleep(0.01)

    finally:
        leftmotor.set_dps(0)
        rightmotor.set_dps(0)

def travel_short(waypoint):
    # Convert the waypoint from grid coordinates to centimetres.
    target_x = waypoint[0] * TILE
    target_y = waypoint[1] * TILE

    # Calculate displacement from the current position.
    dx = target_x - x
    dy = target_y - y

    # Calculate the straight-line distance to the waypoint.
    distance = math.hypot(dx, dy)

    # Already at the waypoint.
    if distance < 1.0:
        return

    # Calculate heading:
    # 0 degrees points along +y, increasing clockwise toward +x.
    target_heading = math.degrees(math.atan2(dx, dy)) % 360.0

    # Turn using the minimum angle.
    execute_turn(target_heading)

    # Drive directly to the waypoint.
    drive_straight_distance(distance, target_heading)
try:
    travel_safe([1, 1])

finally:
    leftmotor.set_dps(0)
    rightmotor.set_dps(0)
    reset_brick()
