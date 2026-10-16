from utils.brick import EV3ColorSensor, reset_brick, wait_ready_sensors, Motor, time
import math

# --- Constants & Configuration ---
TILE_SIZE = 30.48  # cm per tile as specified in lab instructions[cite: 1]
r = 2.2            # cm (wheel radius)
b = 9.8            # cm (track width)


left_motor = Motor("B")
right_motor = Motor("C")

# Initialize encoder tracking for odometry
left_ep = left_motor.get_encoder()
right_ep = right_motor.get_encoder()

# Global state variables (x, y in cm, theta in degrees)
x = 0.0
y = 0.0
theta = 0.0  # 0 degrees points straight along the y-axis

# Waypoints provided in grid units (e.g., (1, 2) means 1 tile right, 2 tiles up)
# During the demo, you can modify or replace this array with the TA's specified points.
grid_waypoints = [
    (1, 1),
    (1, 2),
    (2, 0)
]

LEFT_THRESHOLD =25         # EV3 left sensor (get_red below this = black)
RIGHT_THRESHOLD =6         # NXT right sensor (get_value below this = black)


color_sensor_left = EV3ColorSensor(2)    # EV3, port 2
#color_sensor_right = NXTColorSensor(1)	 #NXT, port 1




wait_ready_sensors()



def update_odometer():
    """Continuously tracks position (x, y) and heading (theta) using wheel encoders."""
    global x, y, theta, left_ep, right_ep
    
    left_ec = left_motor.get_encoder()
    right_ec = right_motor.get_encoder()
    
    Del = left_ec - left_ep
    Der = right_ec - right_ep
    
    dl = (Del * 2 * math.pi * r) / 360
    dr = (Der * 2 * math.pi * r) / 360
    dd = (dl + dr) / 2
    
    # Protect math.asin from domain errors due to tiny float drifts
    val = (dl - dr) / b
    val = max(-1.0, min(1.0, val))
    
    dtheta_rad = math.asin(val)
    dtheta = (180 / math.pi) * dtheta_rad
    dx = dd * math.sin(dtheta_rad)
    dy = dd * math.cos(dtheta_rad)
    
    x += dx
    y += dy
    theta += dtheta
    
    # Normalize theta to 0 - 360 range
    theta = theta % 360
    if theta < 0:
        theta += 360.0
        
    left_ep = left_ec
    right_ep = right_ec

def get_shortest_angle(target_theta, current_theta):
    """Ensures the robot takes the minimal angle needed to turn (-180 to 180 degrees)."""
    delta_theta = target_theta - current_theta
    while delta_theta > 180:
        delta_theta -= 360
    while delta_theta < -180:
        delta_theta += 360
    return delta_theta

def execute_turn(target_delta_theta):
    """Turns the robot by a specified angle while running live odometry."""
    left_start = left_motor.get_encoder()
    speed = 20
    left_motor.set_dps(speed)
    right_motor.set_dps(-speed)
    
    turned_degrees = 0
    while abs(turned_degrees) < abs(target_delta_theta):
        update_odometer()
        encoder_diff = left_motor.get_encoder() - left_start
        turned_degrees = encoder_diff * (2 * r / b)
        time.sleep(0.01)
        
    left_motor.set_dps(0)
    right_motor.set_dps(0)

def short_drive_distance(target_distance):
    left_start = left_motor.get_encoder()
    speed = -30
    left_motor.set_dps(speed)
    right_motor.set_dps(speed)
    
    traveled_distance = 0
    while traveled_distance < target_distance:
        update_odometer()
        encoder_diff = left_motor.get_encoder() - left_start
        traveled_distance = (encoder_diff / 360.0) * (2 * math.pi * r)
        time.sleep(0.01) # for CPu of PI
        
    # Stop motors when target distance is reached
    left_motor.set_dps(0)
    right_motor.set_dps(0)
    
    
    
def check_for_black():
    
    count= 0
    left_black = color_sensor_left.get_red() <= LEFT_THRESHOLD #true if black
    
    while count<10 and not(left_black) :
        #check the left
        update_odometer()
        left_motor.set_dps(-30)
        right_motor.set_dps(0)
        count+=1
        
        left_black = color_sensor_left.get_red() <= LEFT_THRESHOLD #true if black
        time.sleep(0.01)
        if left_black:
            return
    
    
    #move back to center line and check the right:
        count =0
    while count<(2*10) and not(left_black) :
        update_odometer()
        left_motor.set_dps(0)
        right_motor.set_dps(-30)
        count+=1
        
        left_black = color_sensor_left.get_red() <= LEFT_THRESHOLD #true if black
        time.sleep(0.01)
        
        if left_black:
            return
        
   #pivot back to center if not found at right:
    count =0
    while count < 10 and not(left_black):
        update_odometer()
        left_motor.set_dps(-30)
        right_motor.set_dps(0)
        
        left_black = color_sensor_left.get_red() <= LEFT_THRESHOLD #true if black
        time.sleep(0.01)
        
        
def safe_drive_distance(target_distance):
    """Drives straight for a specified distance in cm while running live odometry."""
    left_start = left_motor.get_encoder()
    speed = -30
    left_motor.set_dps(speed)
    right_motor.set_dps(speed)
    
    traveled_distance = 0
    while traveled_distance < target_distance:
        update_odometer()
        encoder_diff = left_motor.get_encoder() - left_start
        traveled_distance = (encoder_diff / 360.0) * (2 * math.pi * r)
        
        # Optional: You can check your color sensors here during the Safest Path 
        # to trigger grid-line corrections if crossing tile boundaries.
       
       
        '''
for two sensors


        left_black = color_sensor_left.get_red() <= LEFT_THRESHOLD #true if black
        right_black = color_sensor_right.get_value()[0] <= RIGHT_THRESHOLD # true if black
        
        if (left_black and (not right_black)):
            #turn right a bit
            leftmotor.set_dps(0)
            rightmotor.set_dps(-30)
            
        elif (right_black and (not left_black)):
            #turn left a little
            left_motor.set_dps(-30)
            right_motor.set_dps(0)
        else:
            #no black is caught, or both lines are black inch fwd
            left_motor.set_dps(-30)
            right_motor.set_dps(-30)
        '''
        
        
        
        #for one sensor: put it in the sensor
        
    
        left_black = color_sensor_left.get_red() <= LEFT_THRESHOLD #true if black
        #right_black = color_sensor_right.get_value()[0] <= RIGHT_THRESHOLD # true if black
        
        
        #on good path:
        if left_black:
            left_motor.set_dps(-30)
            right_motor.set_dps(-30)
            
        elif not (left_black):
            #stop motion of motors:
            left_motor.set_dps(0)
            right_motor.set_dps(0)
            check_for_black()
             #resume fwd drive bc we found blakc line
            left_motor.set_dps(speed)
            right_motor.set_dps(speed)


        
        
        time.sleep(0.01)
        
    left_motor.set_dps(0)
    right_motor.set_dps(0)

        
def navigate_shortest_path(target_gx, target_gy):
    """Navigates directly to a waypoint using the shortest diagonal distance and minimal angle turn."""
    global theta
    
    target_x = target_gx * TILE_SIZE
    target_y = target_gy * TILE_SIZE
    
    dx = target_x - x
    dy = target_y - y
    target_theta = math.degrees(math.atan2(dx, dy))
    
    delta_theta = get_shortest_angle(target_theta, theta)
    print(f"Shortest Path: Turning {delta_theta:.2f}° towards ({target_gx}, {target_gy})...")
    execute_turn(delta_theta)
    theta = target_theta  
    
    distance = math.sqrt(dx**2 + dy**2)
    print(f"Driving straight for {distance:.2f} cm...")
    short_drive_distance(distance)

def navigate_safest_path(target_gx, target_gy):
    """Navigates to a waypoint strictly along X and Y axes sequentially[cite: 1]."""
    global theta
    
    target_x = target_gx * TILE_SIZE
    target_y = target_gy * TILE_SIZE
    
    # --- Step 1: Move along X-axis ---
    dx = target_x - x
    if abs(dx) > 1.0:
        target_theta_x = 90 if dx > 0 else 270
        delta_theta = get_shortest_angle(target_theta_x, theta)
        
        print(f"Safest Path [X-Move]: Turning {delta_theta:.2f}°...")
        execute_turn(delta_theta)
        theta = target_theta_x
        
        print(f"Driving X-distance: {abs(dx):.2f} cm...")
        safe_drive_distance(abs(dx))
        
    # --- Step 2: Move along Y-axis ---
    dy = target_y - y
    if abs(dy) > 1.0:
        target_theta_y = 0 if dy > 0 else 180
        delta_theta = get_shortest_angle(target_theta_y, theta)
        
        print(f"Safest Path [Y-Move]: Turning {delta_theta:.2f}°...")
        execute_turn(delta_theta)
        theta = target_theta_y
        
        print(f"Driving Y-distance: {abs(dy):.2f} cm...")
        safe_drive_distance(abs(dy))

# --- Main Program Execution ---
try:
    print("Waiting for hardware initialization...")
    wait_ready_sensors(True)
    print("Hardware ready.")
    
    # Interactive shell input to choose navigation strategy
    print("\nSelect Navigation Mode:")
    print("  1: Shortest path (diagonal)")
    print("  2: Safest path (axis-aligned X then Y)")
    choice = input("Enter choice (type 'shortest' or 'safest'): ").strip().lower()
    
    if choice in ['1', 'shortest', 'sh']:
        mode = 'shortest'
    elif choice in ['2', 'safest', 'safe']:
        mode = 'safest'
    else:
        print("Invalid choice recognized. Defaulting to 'shortest' path.")
        mode = 'shortest'
        
    print(f"\nStarting Navigation in [{mode.upper()}] mode...")
    
    for point in grid_waypoints:
        grid_x, grid_y = point
        print(f"\n--- Heading to grid waypoint: {point} ---")
        
        if mode == 'shortest':
            navigate_shortest_path(grid_x, grid_y)
        else:
            navigate_safest_path(grid_x, grid_y)
            
    print("\nAll waypoints reached successfully!")

except KeyboardInterrupt:
    print("\nProgram interrupted by user (Ctrl+C). Stopping motors safely...")
    left_motor.set_dps(0)
    right_motor.set_dps(0)
    reset_brick()
    print("Brick reset complete.")