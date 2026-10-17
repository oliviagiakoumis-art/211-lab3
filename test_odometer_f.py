from utils.brick import Motor, reset_brick, wait_ready_sensors
import time
import math

#Constants from lab 2
RADIUS = 2.2            
TRACK_WIDTH = 10.5 

leftmotor = Motor("B")
rightmotor = Motor("C")
SPEED = -30
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
try:
    while True:
        update_odometer()
        print(f"X: {x:.2f} cm | Y: {y:.2f} cm | Theta: {theta:.2f}°", end="\r")
        time.sleep(0.05)
except KeyboardInterrupt:
    print("\nOdometry test ended.")
    reset_brick()
    exit()