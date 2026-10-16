from utils.brick import Motor, wait_ready_sensors, time
import math

r = 2.2  # wheel radius in cm
b = 9.8  # track width in cm

left_motor = Motor("B")
right_motor = Motor("C")

left_ep = left_motor.get_encoder()
right_ep = right_motor.get_encoder()

x = 0.0
y = 0.0
theta = 0.0

wait_ready_sensors(True)
print("Odometry live tracker. Push the robot or drive it manually...")

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
    dtheta = math.degrees(dtheta_rad)
    
    x += dd * math.sin(dtheta_rad)
    y += dd * math.cos(dtheta_rad)
    theta = (theta + dtheta) % 360
    
    left_ep = left_ec
    right_ep = right_ec

try:
    while True:
        update_odometer()
        print(f"X: {x:.2f} cm | Y: {y:.2f} cm | Theta: {theta:.2f}°", end="\r")
        time.sleep(0.05)
except KeyboardInterrupt:
    print("\nOdometry test ended.")