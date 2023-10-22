# import packages
import socket
from mfrc522 import SimpleMFRC522
import RPi.GPIO as GPIO
from gpiozero import DistanceSensor, LED, Servo
import RPi.GPIO as GPIO
from time import sleep
import time
from pins import US_SENSOR, SERVO, led
from src import motor_control
from src.rfid_lookup import dictionary
GPIO.setwarnings(False)
import threading
import numpy as np
import socket


# Initialise other devices
servo = Servo(SERVO)
servo.detach() 
GPIO.setmode(GPIO.BCM)
reader = SimpleMFRC522()
stop = False

# Initialise ultrasonics
sensor_front_left = DistanceSensor(echo=US_SENSOR['front']['ECHO'], trigger=US_SENSOR['front']['TRIG'], threshold_distance= 0.2)
sensor_side_forward = DistanceSensor(echo=US_SENSOR['side_forward']['ECHO'], trigger=US_SENSOR['side_forward']['TRIG'])
sensor_side_rear = DistanceSensor(echo=US_SENSOR['side_rear']['ECHO'], trigger=US_SENSOR['side_rear']['TRIG'])
sensor_back_left = DistanceSensor(echo=US_SENSOR['back_left']['ECHO'], trigger=US_SENSOR['back_left']['TRIG'], threshold_distance = 0.05)
sensor_front_right = DistanceSensor(echo=US_SENSOR['back_right']['ECHO'], trigger=US_SENSOR['back_right']['TRIG'], threshold_distance = 0.05)

# Initialise other variables
front_flag = False 
back_flag = False 
counter = 0
counter_alligning = 0
DISTANCE_TOLERANCE = 0.003  # in meters, for perpendicular check

# Bluetooth setup
# s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
# host_address = 'D8:3A:DD:2A:06:7E'  # Fill in your host address here
# s.connect((host_address, 1))
goal_dest = 0

INIT_FLAG = True

connected = 0
print("starting robot \nconnecting to bluetooth...")
try:
    s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    host_address = 'D8:3A:DD:2A:06:7E' # Fill in host address here
    s.connect((host_address,1))
    print("connected")
    connected=1
except OSError:
    print("OSError:", OSError)
    print("not connected, continuing in single robot slay era")
    pass
finally:
    pass

##################################################################################################

def side_detected():
    global side_flag, counter
    print('side detected')
    side_flag = True

def front_detected():
    stop_motors()
    global front_flag, counter, goal_dest,connected,other_robot, INIT_FLAG
    other_robot = -1
    counter = counter + 1 
    if not (counter==2 and goal_dest==3):
        time.sleep(0.25)
    stop_motors()
    if counter == 4:
        counter = 0
    if counter == 2:
        if INIT_FLAG:
            print("init flag is false")
            INIT_FLAG=False
    front_flag = True 

def comm_message():
    global connected,other_robot,counter
    if connected:
        stop_motors()
        while other_robot == -1:
            s.send(bytes(str(counter),'UTF-8'))
            other_robot = int(s.recv(1024))

def back_detected():
    global back_flag
    print('back detected')
    back_flag = True

def get_distances_side():
    """Get distances from both sensors."""
    distance1 = sensor_side_forward.distance
    distance2 = sensor_side_rear.distance 
    return distance1, distance2

def get_distances_front():
    """Get distances from rear sensors."""
    distance1 = sensor_front_right.distance
    distance2 = sensor_front_left.distance
    return distance1, distance2

def receive_package():
    sleep(0.2)
    print("open tray")
    servo.max()
    sleep(1.3)
    servo.detach()
    dest = 0
    print("waiting for package & reading rfid...")
    while dest == 0:
        id = reader.read_id()
        dest = dictionary[str(id)]
    print("package received & goal dest:", dest)
    print("stand back, robot about to drive")
    sleep(2)
    return dest

def deposit_package():
    print("ready to deposit")
    sleep(0.3)
    servo.min() # run servo
    sleep(1.4)
    servo.detach()
    print("desposited package")
    sleep(0.3)
    return

def robot_alignment_side(offset=0):
    global stop
    stop = True
    """Rotate the robot until it becomes perpendicular to the wall, using side 2 ultrasonics."""
    print("correcting side alignment...")
    while True:
        distance1, distance2 = get_distances_side()
        distance2 += offset
        if abs(distance1 - distance2) <= DISTANCE_TOLERANCE:
            stop_motors()
            print("side aligned")
            break
        elif distance1 < distance2:
            motor_control.run(speed1=0.1, speed2=-0.1)
        else:
            motor_control.run(speed1=-0.1, speed2=0.1)
        sleep(0.1)

def robot_alignment_front():
    """Rotate the robot until it becomes perpendicular to the wall, using front 2 ultrasonics."""
    print("correcting front alignment...")    
    while True:
        distance1, distance2 = get_distances_front()
        if abs(distance1 - distance2) <= DISTANCE_TOLERANCE:
            motor_control.run(speed1=0, speed2=0)  # Stop 
            print("front aligned")
            break
        elif distance1 < distance2:
            motor_control.run(speed1=0.1, speed2=-0.1)
        else:
            motor_control.run(speed1=-0.1, speed2=0.1) 
        sleep(0.1)

def robot_rotate_90deg():
    global counter_alligning,other_robot, INIT_FLAG
    print("robot rotate 90deg")
    # stop_motors()
    # time.sleep(1)
    motor_control.run(speed1 = 1, speed2 = -1)
    sleep(0.6)
    stop_motors()
    sleep(1)   
    counter_alligning = 0
    print(counter_alligning)
    other_robot=-1

    if connected:
        stop_motors()
        while (other_robot == -1 or (abs(counter-other_robot)%2 == 0)) and not INIT_FLAG:
            s.send(bytes(str(counter),'UTF-8'))
            other_robot = int(s.recv(1024))
            
    print(f"our counter = {counter}")
    print(f"other robot counter = {other_robot}")
    other_robot=-1
    stop = False

def drive(speed1,speed2):
    motor_control.run(speed1, speed2)

def stop_motors():
    motor_control.run(speed1=0, speed2=0)

def adjust_position_from_wall(current_distance, target_distance, forward_speed, backward_speed):
    robot_alignment_side()
    if current_distance < target_distance:
        while sensor_back_left.distance * 100 < target_distance:
            motor_control.run(speed1=forward_speed, speed2=forward_speed)
            sleep(0.1)
    elif current_distance > target_distance:
        while sensor_back_left.distance * 100 > target_distance:
            motor_control.run(speed1=backward_speed, speed2=backward_speed)
            sleep(0.1)
    robot_alignment_side()
    stop_motors()

def adjust_position_from_wall_front(current_distance, target_distance, forward_speed, backward_speed):
    robot_alignment_side()
    if current_distance < target_distance:
        while (sensor_front_left.distance*100 + sensor_front_right.distance*100)/2 < target_distance:
            motor_control.run(speed1=backward_speed, speed2=backward_speed)
            sleep(0.1)
    elif current_distance > target_distance:
        while (sensor_front_left.distance*100 + sensor_front_right.distance*100)/2 > target_distance:
            motor_control.run(speed1=forward_speed, speed2=forward_speed)
            sleep(0.1)
    robot_alignment_side()
    stop_motors()


##################################################################################################
# def periodic_alligning():
#     while not(stop):
#         robot_alignment_side()
#         sleep(2)

# Create a thread for periodic alignment
# alignment_thread = threading.Thread(target=periodic_alligning)
# alignment_thread.daemon = False  # Keep it running in the background
# alignment_thread.start()

def main_sequence():
    global front_flag, counter, back_flag, goal_dest, counter_alligning, other_robot, INIT_FLAG
    # print('ebcuveii')
    # drive straight
    # while True:
    #     drive(0.7,0.7)
    #     print("chekcne")
    if counter == 1:
        drive(0.7,0.7)
    elif counter == 2 and goal_dest == 3:
        drive(0.7,0.7)
    else:
        drive(0.9,0.9)

    # when robot reaches a wall/corner
    if front_flag == True:

        if not (counter == 2 and goal_dest == 3):
            robot_rotate_90deg()
            robot_alignment_side(0.02)

        if counter == 1:
            if goal_dest == 1:
                robot_alignment_side()
                drive(0.7,0.7)
                sleep(0.4)
                stop_motors()
                robot_alignment_side()
                adjust_position_from_wall(sensor_back_left.distance*100, 12, 0.2, -0.2)
                drive(-0.2,-0.2)
                sleep(0.4)
                stop_motors()
                robot_alignment_side(-0.01)
                deposit_package()
                robot_alignment_side(0.01)

            elif goal_dest == 2:
                adjust_position_from_wall(sensor_back_left.distance*100, 42, 0.4, -0.4)
                sleep(0.3)
                adjust_position_from_wall_front(sensor_front_left.distance*100, 45, 0.4, -0.4)
                deposit_package()
                robot_alignment_side(0.02)

            elif goal_dest == 3:
                adjust_position_from_wall_front(sensor_front_left.distance*100, 30, 0.5, -0.5)
                robot_alignment_side(0.01)

        elif counter == 2:
            if goal_dest == 3:
                robot_alignment_side()
                drive(0.3,0.3)
                sleep(1.2)
                robot_alignment_side(0.02)
                deposit_package()
                robot_rotate_90deg()
                robot_alignment_side(0.02)
                counter = 2

    if counter == 4:
        stop_motors()
        sleep(0.5)
        robot_rotate_90deg()
        counter = 0
        stop_motors()
        robot_alignment_side()
        adjust_position_from_wall(sensor_back_left.distance*100, 10, 0.2, -0.2)

        print("At loading zone")
        goal_dest = receive_package()
        counter = 0
        robot_alignment_side()
                
    # reset flags 
    front_flag = False
    back_flag = False 


# Attach the callback function
sensor_front_left.when_in_range = front_detected
sensor_side_forward.when_in_range = side_detected 
sensor_side_rear.when_in_range = side_detected 
sensor_front_right.when_in_range = front_detected 
sensor_back_left.when_in_range = back_detected


try:
    sleep(1)
    goal_dest = receive_package()
    while True:
        main_sequence()
        sleep(0.1)
except KeyboardInterrupt:
    print("Program terminated.")
finally:
    # Cleanup
    motor_control.encoder1.close()
    motor_control.encoder2.close()
    motor_control.motor1_pwm1.close()
    motor_control.motor1_pwm2.close()
    motor_control.motor2_pwm1.close()
    motor_control.motor2_pwm2.close()
    sensor_front_left.close()
    sensor_side_forward.close()
    sensor_side_rear.close()
    sensor_back_left.close()
    sensor_front_right.close()