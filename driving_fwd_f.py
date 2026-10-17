from utils.brick import Motor, reset_brick, wait_ready_sensors, EV3ColorSensor
import time
import math

#Constants from lab 2
RADIUS = 2.2            
TRACK_WIDTH = 10.5 
SPEED = -30
TILE = 30.48  
THRESHOLD = 25
STEPS = 30
CORRECTION = 0.5

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
    slow = SPEED * CORRECTION
    if direction == "right":      
        leftmotor.set_dps(SPEED)
        rightmotor.set_dps(slow)
    else:
        leftmotor.set_dps(slow)
        rightmotor.set_dps(SPEED)

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
            leftmotor.set_dps(SPEED)
            rightmotor.set_dps(SPEED)
            update_odometer()
            print(f"X: {x:.2f} cm | Y: {y:.2f} cm | Theta: {theta:.2f}°", end="\r")
            time.sleep(0.01)
    leftmotor.set_dps(0)
    rightmotor.set_dps(0)

try:
    print("Testing forward drive with live recovery over 1 tile (30.48 cm)...")
    drive_forward(TILE, "y")
    test = input("\nPress Enter to continue to the color correction test...")
    drive_forward_correct(TILE, "y")

finally:
    reset_brick()

    