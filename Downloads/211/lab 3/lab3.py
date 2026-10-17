from utils.brick import Motor, EV3ColorSensor, NXTColorSensor, reset_brick, wait_ready_sensors
import time
import math

TILE_SIZE = 30.48  
PATH_TYPE = "SAFE"
WAYPOINTS = [(1,1), (1,2), (2,0)]
RIGHT_THRESHOLD = 3   
RADIUS = 2.2            
TRACK_WIDTH = 10.0 

leftmotor = Motor("C")
rightmotor = Motor("B") 
colorsensor = NXTColorSensor(1)

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


def travel_safe(waypoint):
   move_fwd(waypoint[0] * TILE_SIZE, "x")
   turn(90)
   move_fwd(waypoint[1] * TILE_SIZE, "y")


def travel_short(waypoint):
    angle = math.degrees(math.atan2(delta_y, delta_x)) - theta
    turn(angle)
    leftmotor.set_dps(30)
    rightmotor.set_dps(30)

def correct_path(left, right):
    if (colorsensor.get_value()[0] > RIGHT_THRESHOLD):
        left.set_dps(20)
        right.set_dps(30)
        update_odometer()
        time.sleep(0.01)

def turn(turn_angle):
    
    initial= theta
    leftmotor.set_dps(30)
    rightmtoor.set_dps(-30)
    
    while(theta< initial + turn angle):
        update_odometer()
        time.sleep(0.01)

def move_fwd(d, a):
    if a == "x":
        initial = x #if in x-dir,,if in y, change this logic to y
        leftmotor.set_dps(30)
        rightmotor.set_dps(30)

        while(x < initial + d):
            update_odometer()
            time.sleep(0.01)
    else:
        initial = y #if in x-dir,,if in y, change this logic to y
        leftmotor.set_dps(30)
        rightmotor.set_dps(30)

        while(y < initial + d):
            update_odometer()
            time.sleep(0.01)
    
    leftmotor.set_dps(0) #for when you've reached necessary destination
    rightmotor.set_dps(0)   

if __name__ == "main":
    try:
        for waypoint in WAYPOINTS:
            if PATH_TYPE == "SAFE":
                travel_safe(waypoint)
                correct_path(leftmotor, rightmotor)
            elif PATH_TYPE == "SHORT":
                travel_short(waypoint)
                
            else:
                print("Invalid PATH_TYPE. Please choose 'SAFE' or 'SHORT'.")
    except BaseException:
        reset_brick()
        exit()
    reset_brick()