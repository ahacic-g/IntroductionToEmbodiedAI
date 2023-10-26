#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, ColorSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
# from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

# ------------------- VARIABLE DEFINITIONS -------------------------
# PID parameters
# for turn control
turn_kp = 1.0
turn_ki = 0.0
turn_kd = 0.0
# for speed control
speed_kp = 0.5
speed_ki = 0.05
speed_kd = 0.01
# other PID constants
target_reflection = 50 # TO BE DEFINED LATER
turn_integral = 0
speed_integral = 0
turn_last_error = 0
speed_last_error = 0

# robot dimensions
wheel_diameter = 82 # diameter of the wheels on the motors
axle_track = 150 # distance between the wheels

# primary speed and angle definitions
turning_speed = 50 # mm/s
turning_angle = 90 # deg
max_speed = 150 # mm/s
gripper_speed = 50 # deg/s
gripper_open_angle = 80 # deg
gripper_close_angle = 0 # deg
search_angle = 45 # deg

# booleans
calibration_state = True
drive_state = True
search_state = False

# states
state_dict = {
    "follow_line":True,
    "turn_left":False,
    "turn_right":False,
    "uphill":False,
    "downhill":False,
    "stop":False,
    "search":False,
    "move_to_target":False,
    "open_gripper":False,
    "close_gripper":False,
    "return":False
}
state = "straight"

# thresholds
black_threshold = 20
mild_hill = 5 # TO BE CORRECTED WHEN TESTED
steep_hill = 15 # TO BE CORRECTED WHEN TESTED


# ------------------- OBJECTS --------------------------------------
ev3 = EV3Brick()
SW = StopWatch()
right_motor = Motor(Port.A)
left_motor = Motor(Port.D)
gripper_motor = Motor(Port.B,Direction.CLOCKWISE) # ADD GEAR RATIO, CHANGE DIRECTION IF NECESSARY
drive_base = DriveBase(left_motor, right_motor, wheel_diameter=wheel_diameter, axle_track=axle_track)
left_light = ColorSensor(Port.S4)
right_light = ColorSensor(Port.S1)
gyro_module = GyroSensor(Port.S2, Direction.COUNTERCLOCKWISE)
ultrasound_module = UltrasonicSensor(Port.S3)


# ------------------- FUNCTION DEFINITIONS -------------------------
# Reading the sensors from the robot
def read_sensors(light_sensor=True, gyro_sensor=True, ultrasound_sensor=True):
    sensor_readings = {}
    if light_sensor == True:
        left = left_light.reflection()
        right = right_light.reflection()
        sensor_readings["left"] = left
        sensor_readings["right"] = right
    if gyro_sensor == True:
        gyro = gyro_module.angle()
        sensor_readings["gyro"] = gyro
    if ultrasound_sensor == True:
        ultrasound = ultrasound_module.distance()
        sensor_readings["ultrasound"] = ultrasound
    return sensor_readings

# calibrate the robot parameters for PID control
def calibrate(angle):
    global target_reflection
    rotation = drive_base.angle()
    rotate = -5
    while rotation < angle:
        sensor_readings = read_sensors(gyro_sensor=False, ultrasound_sensor=False)
        left_sensor = sensor_readings["left"]
        right_sensor = sensor_readings["right"]
        current_reflection = left_sensor + right_sensor
        if target_reflection < current_reflection:
            target_reflection = current_reflection
        if drive_base.angle() <= -angle:
            rotate = 5
        drive_base.drive(0, rotate)
        rotation = drive_base.angle()
    drive_base.turn(-angle)
    # just for the testing purposes
    print("Target reflection: ", target_reflection)
    print("Calibration finished")

# PID control
def pid_control(left_sensor, right_sensor):
    global turn_integral, speed_integral, turn_last_error, speed_last_error
    turn_error = target_reflection - (left_sensor + right_sensor)
    speed_error = target_reflection - (left_sensor + right_sensor)

    turn_integral = turn_integral + turn_error
    speed_integral = speed_integral + speed_error

    turn_derivative = turn_error - turn_last_error
    speed_derivative = speed_error - speed_last_error

    turn = turn_kp * turn_error + turn_ki * turn_integral + turn_kd * turn_derivative
    speed = speed_kp * speed_error + speed_ki * speed_integral + speed_kd * speed_derivative

    turn_last_error = turn_error
    speed_last_error = speed_error

    if speed > max_speed:
        speed = max_speed

    return turn, speed

# Gripper function
def use_gripper(open_gripper=False, close_gripper=False):
    if open_gripper == True and close_gripper == False:
        gripper_motor.run_target(gripper_speed, gripper_open_angle)
    elif close_gripper == True and open_gripper == False:
        gripper_motor.run_target(gripper_speed, gripper_close_angle)
    else:
        print("Error: Gripper can only open or close")

# Search function
def search_for_target():
    search_state = True
    nearest_target = []
    use_gripper(open_gripper=True)
    drive_base.reset()
    drive_base.turn(search_angle)
    while search_state == True:
        ultrasound = read_sensors(light_sensor=False, gyro_sensor=False, ultrasound_sensor=True)
        if nearest_target == []:
            nearest_target.append(drive_base.angle())
            nearest_target.appennd(ultrasound)
        else:
            if ultrasound < nearest_target[1]:
                nearest_target[0] = drive_base.angle()
                nearest_target[1] = ultrasound
        drive_base.turn(-5)
        if drive_base.angle() <= -search_angle:
            search_state = False

    return nearest_target
    
# Move and grab the target
def move_and_grab(target_pos):
    while True: 
        curent_angle = drive_base.angle()
        drive_base.turn(current_angle - target_pos[0])
        drive_base.straight(target_pos[1])
        use_gripper(close_gripper=True)
        # drive back to the line
        drive_base.straight(-target_pos[1])
        drive_base.turn(-target_pos[0])
        calibration_state = True
        break

# check if there is a line
def check_for_line(left_sensor, right_sensor):
    if left_sensor < black_threshold and right_sensor < black_threshold:
        return False
    else:
        return True

# timing the gap
def gap_timing(allowed_time=1000):
    SW.reset()
    while True:
        time = SW.time()
        if time > allowed_time:
            search_state = True
            break

# handling of the gap in the line
def gap_handling(left_sensor, right_sensor):
    pass # currently not implemented - FINISH THIS FUNCTION

# handling the speed during uphill or downhill movement
def hill_speed_handling(speed, angle):
    speed = speed
    if angle < mild_hill and angle > -mild_hill:
        pass
    # uphill handling
    elif angle >= mild_hill and angle < steep_hill:
        speed = speed*1.3
    elif angle >= steep_hill:
        speed = speed*1.5

    # downhill handling
    elif angle <= -mild_hill and angle > -steep_hill:
        speed = speed*0.7
    elif angle <= -steep_hill:
        speed = speed*0.5

    return speed 
        
# function that enables driving of the robot
def drive(speed, turn, drive=True):
    if drive == False:
        drive_base.stop()
    else:
        drive_base.drive(speed, turn)

# start the search for the target state -- FINISH THIS FUNCTION -- CURRENTLY NOT IMPLEMENTED
def start_the_search():
    drive_state = False


# ------------------- MAIN PROGRAM ---------------------------------
while True:
    # ---------- CALIBRATION -------------------
    # calibrate the robot just on the start of the program and when the robot collects the object before returning
    while calibration_state == True:
        calibrate(20)
        calibration_state = False

    # ---------- SENSOR READING AND CALCULATION -------------------
    # read the sensors on every pass
    sensor_values = read_sensors(ultrasound_sensor=False)
    # compute speed and angle with PID and check if we are on the hill on every pass
    angle, speed = pid_control(sensor_values["left"], sensor_values["right"])
    speed = hill_speed_handling(speed, sensor_values["gyro"])

    # add the gap hanglig logic if needed!! --> TESTS FIRST
    # add the sharp 90 degree turns logic if needed!! --> TESTS FIRST

    # ---------- SEARCH AND GRAB -------------------
    # switch the search_state on somehow!! still have to figure that one out
    if check_for_line(sensor_values["left"], sensor_values["right"]) == False:
        # add gap handling logic here
        gap_timing()

    while search_state == True:
        nearest_target = search_for_target()
        move_and_grab(nearest_target)
        break

    # ---------- MOVE THE ROBOT ---------------------
    drive(speed, angle, drive=drive_state)
