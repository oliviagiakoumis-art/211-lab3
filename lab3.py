from utils.brick import Motor, EV3ColorSensor, reset_brick, wait_ready_sensors
import time
import math

TILE = 30.48  
PATH_TYPE = "SAFE" #SHORT or SAFE
WAYPOINTS = [(1,1), (2,2), (2,0)]
THRESHOLD = 35 
RADIUS = 2.2            
TRACK_WIDTH = 10.0 
SPEED = -100
STEPS = 30


leftmotor = Motor("C")
rightmotor = Motor("B") 
colorsensor = EV3ColorSensor(1)

x = 0.0
y = 0.0                                                     
theta = 0.0

leftmotor.reset_encoder()
rightmotor.reset_encoder()
prev_right_encoder = 0
prev_left_encoder = 0

def update_odometer():
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

def execute_turn(target_delta_theta):
    #turn function in both directions, positive for clockwise and negative for counter-clockwise
    initial = theta
    target = initial + target_delta_theta

    if target_delta_theta > 0:
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

    rightmotor.set_dps(0)

def drive_forward(distance, direction):
    if direction == "x":
        target_x = x + distance
        while x < target_x:
            leftmotor.set_dps(SPEED)
            rightmotor.set_dps(SPEED)
            update_odometer()
            print(f"X: {x:.2f} cm | Y: {y:.2f} cm | Theta: {theta:.2f}°", end="\r")
            time.sleep(0.01)  
    else:
        target_y = y + distance
        while y < target_y:
            leftmotor.set_dps(SPEED)
            rightmotor.set_dps(SPEED)
            update_odometer()
            print(f"X: {x:.2f} cm | Y: {y:.2f} cm | Theta: {theta:.2f}°", end="\r")
            time.sleep(0.01)
    leftmotor.set_dps(0)
    rightmotor.set_dps(0)

def steer(direction):
    if direction == "right":      
        leftmotor.set_dps(SPEED*0.25)
        rightmotor.set_dps(0)
    else:
        leftmotor.set_dps(0)
        rightmotor.set_dps(SPEED*0.25)

def correct_path():
    for _ in range(STEPS):
        steer("right")
        update_odometer()
        time.sleep(0.01)
        if colorsensor.get_value()[0] < THRESHOLD:
            return
    while colorsensor.get_value()[0] >= THRESHOLD:
        steer("left")
        update_odometer()
        time.sleep(0.01)

def drive_forward_correct(distance, direction):
    if direction == "x":
        target_x = x + distance
        while x < target_x:
            if colorsensor.get_value()[0] < THRESHOLD:
                leftmotor.set_dps(SPEED)
                rightmotor.set_dps(SPEED)
            else:
                correct_path()  
            update_odometer()
            print(f"X: {x:.2f} cm | Y: {y:.2f} cm | Theta: {theta:.2f}°", end="\r")
            time.sleep(0.01)  
    else:
        target_y = y + distance
        while y < target_y:
            if colorsensor.get_value()[0] < THRESHOLD:
                leftmotor.set_dps(SPEED)
                rightmotor.set_dps(SPEED)
            else:
                correct_path()  
            update_odometer()
            print(f"X: {x:.2f} cm | Y: {y:.2f} cm | Theta: {theta:.2f}°", end="\r")
            time.sleep(0.01) 
    leftmotor.set_dps(0)
    rightmotor.set_dps(0)

def travel_safe(waypoint):
   drive_forward_correct(waypoint[0] * TILE, "x")
   execute_turn(90)
   drive_forward_correct(waypoint[1] * TILE, "y")


def travel_short(waypoint):
    angle = math.degrees(math.atan2(waypoint[0], waypoint[1])) -theta
    execute_turn(angle)
    drive_forward(math.hypot(waypoint[0], waypoint[1]), "y")  

if __name__ == "main":
    try:
        for waypoint in WAYPOINTS:
            if PATH_TYPE == "SAFE":
                travel_safe(waypoint)
            elif PATH_TYPE == "SHORT":
                travel_short(waypoint)
                
            else:
                print("Invalid PATH_TYPE. Please choose 'SAFE' or 'SHORT'.")
    except BaseException:
        reset_brick()
        exit()
    reset_brick()