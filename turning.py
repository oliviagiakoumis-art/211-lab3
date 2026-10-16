from utils.brick import Motor, wait_ready_sensors, time
import math

r = 2.2
b = 9.8

left_motor = Motor("B")
right_motor = Motor("C")

left_ep = left_motor.get_encoder()
right_ep = right_motor.get_encoder()
x, y, theta = 0.0, 0.0, 0.0

wait_ready_sensors(True)

def update_odometer():
    global x, y, theta, left_ep, right_ep
    left_ec = left_motor.get_encoder()
    right_ec = right_motor.get_encoder()
    Del = left_ec - left_ep
    Der = right_ec - right_ep
    dl = (Del * 2 * math.pi * r) / 360
    dr = (Der * 2 * math.pi * r) / 360
    dd = (dl + dr) / 2
    val = max(-1.0, min(1.0, (dl - dr) / b))
    dtheta_rad = math.asin(val)
    x += dd * math.sin(dtheta_rad)
    y += dd * math.cos(dtheta_rad)
    theta = (theta + (180 / math.pi) * dtheta_rad) % 360
    left_ep = left_ec
    right_ep = right_ec

def get_shortest_angle(target_theta, current_theta):
    delta_theta = target_theta - current_theta
    while delta_theta > 180:
        delta_theta -= 360
    while delta_theta < -180:
        delta_theta += 360
    return delta_theta

def execute_turn(target_delta_theta):
    left_start = left_motor.get_encoder()
    speed = 20
    left_motor.set_power(speed)
    right_motor.set_power(-speed)
    
    turned_degrees = 0
    while abs(turned_degrees) < abs(target_delta_theta):
        update_odometer()
        encoder_diff = left_motor.get_encoder() - left_start
        turned_degrees = encoder_diff * (2 * r / b)
        time.sleep(0.01)
        
    left_motor.set_power(0)
    right_motor.set_power(0)

print("Testing a 90-degree turn...")
execute_turn(90)
print("Turn complete.")