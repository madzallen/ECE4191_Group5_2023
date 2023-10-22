"""
File that contains all pin assignments for the raspberry pi and connected modules.
Referenced throughout project.
Useful site: https://pinout.xyz/pinout/pin16_gpio23/
Created: 14/08/2023
"""

# load packages
import RPi.GPIO as GPIO

# Board numbering mode
GPIO.setmode(GPIO.BCM)

# MOTOR CONTROL
# left motor
#PIN 12 IS ENABLE PIN 1 IS PWM1 AND PIN 16 IS PWM 2
m1_enable  = 12 #swapped 
e1_a = 20
e1_b = 21
m1_pwm1 = 1 #swapped
m1_pwm2 = 16

# right motor
#PIN 26 IS ENABLE PIN 13 IS PWM1 AND PIN 19 IS PWM2
m2_enable  = 26 #swapped
e2_a = 5
e2_b = 6
m2_pwm1 = 13 #swapped
m2_pwm2 = 19

# ultrasonic sensors # working, tested on 11.10.23 (DO NOT CHANGE)
US_SENSOR = {
    'front': { # FRONT SENSOR
        'TRIG': 27,
        'ECHO': 24
    },
    'side_forward': { # SIDE FORWARD
        'TRIG': 23,
        'ECHO': 14
    },
    'side_rear': { # SIDE REAR
        'TRIG': 18,
        'ECHO': 15
    },
    'back_left': { # BACK LEFT (ABOVE)
        'TRIG': 3,
        'ECHO': 4
    },
    'back_right': { # BACK RIGHT (ABOVE)
        'TRIG': 2,
        'ECHO': 17
    }
}

# servo motor
SERVO = 22

# RFID reader, built into MFRC522 package as default
SDA  = 8
SCK  = 11
MOSI = 10
MISO = 9
RST  = 25