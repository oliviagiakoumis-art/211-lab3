from utils.brick import ColorSensor, Motor, wait_ready_sensors, time
import math

# --- Constants & Configuration ---
TILE_SIZE = 30.48  # cm per tile
r = 2.2            # cm (wheel radius)
b = 9.8            # cm (track width)

left_motor = Motor("B")
right_motor = Motor("C")

# Initialize encoder tracking for odometry
left_ep = left_motor.get_encoder()
right_ep = right_motor.get_encoder()

# Global state variables
x = 0.0
y = 0.0
theta = 0.0

LEFT_THRESHOLD = 25  # EV3 left sensor threshold
color_sensor_left = ColorSensor("2")  # Adjust port string if needed ("1" or "2")

wait_ready_sensors(True)


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
    
    val = (dl - dr) / b
    val = max(-1.0, min(1.0, val))
    
    dtheta_rad = math.asin(val)
    dtheta = (180 / math.pi) * dtheta_rad
    dx = dd * math.sin(dtheta_rad)
    dy = dd * math.cos(dtheta_rad)
    
    x += dx
    y += dy
    theta += dtheta
    
    theta = theta % 360
    if theta < 0:
        theta += 360.0
        
    left_ep = left_ec
    right_ep = right_ec


def check_for_black():
    count = 0
    left_black = color_sensor_left.get_red() <= LEFT_THRESHOLD
    
    while count < 10 and not(left_black):
        # check the left
        update_odometer()
        left_motor.set_dps(-30)
        right_motor.set_dps(0)
        count += 1
        
        # update left_black
        left_black = color_sensor_left.get_red() <= LEFT_THRESHOLD
        time.sleep(0.01)
        
        if left_black:
            return
    
    # move back to center line and check the right:
    count = 0
    while count < (2 * 10) and not(left_black):
        update_odometer()
        left_motor.set_dps(0)
        right_motor.set_dps(-30)
        count += 1
        
        # update left_black
        left_black = color_sensor_left.get_red() <= LEFT_THRESHOLD
        time.sleep(0.01)
        if left_black:
            return
        
    # pivot back to center if not found at right:
    count = 0
    while count < 10 and not(left_black):
        update_odometer()
        left_motor.set_dps(-30)
        right_motor.set_dps(0)
        left_black = color_sensor_left.get_red() <= LEFT_THRESHOLD 
        time.sleep(0.01)


def drive_distance(target_distance):
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
        
        left_black = color_sensor_left.get_red() <= LEFT_THRESHOLD  # true if black
        
        # on good path:
        if left_black:
            left_motor.set_dps(-30)
            right_motor.set_dps(-30)
        else: 
            # stop motion of motors:
            left_motor.set_dps(0)
            right_motor.set_dps(0)
            check_for_black()
            # resume fwd drive bc we found black line
            left_motor.set_dps(speed)
            right_motor.set_dps(speed)

        time.sleep(0.01)
        
    left_motor.set_dps(0)
    right_motor.set_dps(0)


try:
    print("Testing forward drive with live recovery over 1 tile (30.48 cm)...")
    drive_distance(TILE_SIZE)
    print("Test finished successfully!")

except KeyboardInterrupt:
    print("\nTest interrupted by user.")
    left_motor.set_dps(0)
    right_motor.set_dps(0)