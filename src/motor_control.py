from gpiozero import PWMOutputDevice, RotaryEncoder, DigitalOutputDevice
from time import sleep
import math 
from pins import m1_enable, m1_pwm2, e1_a, e1_b, m1_pwm1
from pins import m2_enable, m2_pwm2, e2_a, e2_b, m2_pwm1

# Initialize motors and global variables

#Initalize the Enable Pin
m1_enable_pin = DigitalOutputDevice(pin=m1_enable, active_high=True)
m2_enable_pin = DigitalOutputDevice(pin=m2_enable, active_high=True)

m1_enable_pin.on()
m2_enable_pin.on()

#Define the PWM Outputs
motor1_pwm1 = PWMOutputDevice(pin=m1_pwm1)
motor1_pwm2 = PWMOutputDevice(pin=m1_pwm2)

motor2_pwm1 = PWMOutputDevice(pin=m2_pwm1)
motor2_pwm2 = PWMOutputDevice(pin=m2_pwm2)

# Initialize encoders
encoder1 = RotaryEncoder(a=e1_a, b=e1_b)
encoder2 = RotaryEncoder(a=e2_a, b=e2_b)

encoder1_value = 0
encoder2_value = 0

# Constants
GEAR_RATIO = 75.0
ENCODER_COUNTS_PER_REVOLUTION = 12.0
WHEEL_DIAMETER = 0.055 # metres 
TOTAL_COUNTS = ENCODER_COUNTS_PER_REVOLUTION * GEAR_RATIO
CIRCUMFERENCE = math.pi * WHEEL_DIAMETER
DISTANCE_PER_PULSE = CIRCUMFERENCE / TOTAL_COUNTS
RADIUS = WHEEL_DIAMETER/2 

def encoder1_callback(position):
    global encoder1_value
    encoder1_value += 1

def encoder2_callback(position):
    global encoder2_value
    encoder2_value += 1

encoder1.when_rotated = encoder1_callback
encoder2.when_rotated = encoder2_callback

def run(speed1, speed2):
    if speed1 > 0:
        motor1_pwm1.value = abs(speed1)
        motor1_pwm2.value = abs(0)

    else:
        motor1_pwm2.value = abs(speed1)
        motor1_pwm1.value = abs(0)
       
    if speed2 > 0:
        motor2_pwm1.value = abs(speed2)
        motor2_pwm2.value = abs(0)
    else:
        motor2_pwm2.value = abs(speed2)
        motor2_pwm1.value = abs(0)
def stop_motors():
    motor1_pwm2.value = abs(0)
    motor1_pwm1.value = abs(0)
    motor2_pwm2.value = abs(0)
    motor2_pwm1.value = abs(0)

