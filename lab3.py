from utils.brick import Motor, EV3ColorSensor, reset_brick, wait_ready_sensors
import time
import math

TILE = 30.48
PATH_TYPE = "SAFE"  # SHORT or SAFE
WAYPOINTS = [(1, 1), (2, 2), (2, 0)]
THRESHOLD = 35
RADIUS = 2.2
TRACK_WIDTH = 10.0
SPEED = -100
STEPS = 30

leftmotor = Motor("C")
rightmotor = Motor("B")
colorsensor = EV3ColorSensor(1)
wait_ready_sensors()

x = 0.0
y = 0.0
theta = 0.0   # degrees, clockwise from +y, NOT wrapped

leftmotor.reset_encoder()
rightmotor.reset_encoder()
prev_right_encoder = 0
prev_left_encoder = 0


def update_odometer():
    #working update_odometer function from lab 2
    global x, y, theta, prev_right_encoder, prev_left_encoder

    current_left_encoder = -leftmotor.get_encoder()
    current_right_encoder = -rightmotor.get_encoder()

    delta_left = current_left_encoder - prev_left_encoder
    delta_right = current_right_encoder - prev_right_encoder

    delta_distance_left = (delta_left / 360.0) * 2 * math.pi * RADIUS
    delta_distance_right = (delta_right / 360.0) * 2 * math.pi * RADIUS

    delta_theta = (delta_distance_left - delta_distance_right) / TRACK_WIDTH
    delta_distance = (delta_distance_left + delta_distance_right) / 2.0

    delta_x = delta_distance * math.sin(math.radians(theta) + delta_theta / 2.0)
    delta_y = delta_distance * math.cos(math.radians(theta) + delta_theta / 2.0)

    x += delta_x
    y += delta_y
    theta = (theta + math.degrees(delta_theta)) % 360.0

    prev_left_encoder = current_left_encoder
    prev_right_encoder = current_right_encoder


def stop():
    leftmotor.set_dps(0)
    rightmotor.set_dps(0)


def execute_turn(delta):
    """Relative turn in degrees. + = clockwise, - = counter-clockwise."""
    target = theta + delta
    if delta > 0:
        leftmotor.set_dps(SPEED)
        rightmotor.set_dps(-SPEED)
        while theta < target:
            update_odometer()
            time.sleep(0.01)
    else:
        leftmotor.set_dps(-SPEED)
        rightmotor.set_dps(SPEED)
        while theta > target:
            update_odometer()
            time.sleep(0.01)
    stop()


def turn_to(heading):
    """Turn to an absolute heading (degrees) by the shortest way."""
    delta = (heading - theta + 180) % 360 - 180
    if abs(delta) > 1:
        execute_turn(delta)


def steer(direction):
    if direction == "right":
        leftmotor.set_dps(SPEED * 0.25)
        rightmotor.set_dps(0)
    else:
        leftmotor.set_dps(0)
        rightmotor.set_dps(SPEED * 0.25)


def correct_path():
    for _ in range(STEPS):
        steer("right")
        update_odometer()
        time.sleep(0.01)
        if colorsensor.get_value()[0] < THRESHOLD:
            return
    for _ in range(2 * STEPS):                     # bounded search
        steer("left")
        update_odometer()
        time.sleep(0.01)
        if colorsensor.get_value()[0] < THRESHOLD:
            return


def drive_forward(distance, correct=False):
    sx, sy = x, y
    while math.hypot(x - sx, y - sy) < distance:
        if not correct or colorsensor.get_value()[0] < THRESHOLD:
            leftmotor.set_dps(SPEED)
            rightmotor.set_dps(SPEED)
        else:
            correct_path()
        update_odometer()
        print(f"X: {x:.2f} cm | Y: {y:.2f} cm | Theta: {theta:.2f}°", end="\r")
        time.sleep(0.01)
    stop()


def travel_safe(waypoint):
    dx = waypoint[0] * TILE - x
    if abs(dx) > 1:
        turn_to(90 if dx > 0 else -90)
        drive_forward(abs(dx), correct=True)

    dy = waypoint[1] * TILE - y
    if abs(dy) > 1:
        turn_to(0 if dy > 0 else 180)
        drive_forward(abs(dy), correct=True)


def travel_short(waypoint):
    dx = waypoint[0] * TILE - x
    dy = waypoint[1] * TILE - y
    turn_to(math.degrees(math.atan2(dx, dy)))
    drive_forward(math.hypot(dx, dy))


if __name__ == "__main__":
    try:
        if PATH_TYPE not in ("SAFE", "SHORT"):
            print("Invalid PATH_TYPE. Please choose 'SAFE' or 'SHORT'.")
        else:
            for waypoint in WAYPOINTS:
                if PATH_TYPE == "SAFE":
                    travel_safe(waypoint)
                else:
                    travel_short(waypoint)
    except BaseException as e:
        print(f"\nError: {e!r}")
    finally:
        reset_brick()