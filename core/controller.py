import threading
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math
from pymavlink import mavutil
import os
import json
import numpy as np
import mss
import datetime
import cv2

# Connect to the Vehicle
print('Connecting to vehicle')
vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)

# Define the stop_requested variable
stop_requested = False
abort_fifo_path = '/tmp/gpt_abort_fifo'
command_fifo_path = '/tmp/gpt_command_fifo'
comin_fifo_path = '/tmp/gpt_comin_fifo'
status_fifo_path = '/tmp/gpt_status_fifo'
statusa_fifo_path = '/tmp/gpt_statusa_fifo'
img_fifo_path = '/tmp/gpt_img_fifo'
imgcont_fifo_path = '/tmp/gpt_imgcont_fifo'
seecont_fifo_path = '/tmp/gpt_seecont_fifo'
imgcontnew_fifo_path = '/tmp/gpt_imgcontnew_fifo'

x, y, width, height = 1875, 1095, 900, 700  # Slightly reduced height



# Your specified folder for storing screenshots
screenshot_folder = "./captures"

def take_screenshot():
    """Capture a screenshot of the specified region and save it to the specified folder."""
    # Ensure the folder exists
    if not os.path.exists(screenshot_folder):
        os.makedirs(screenshot_folder)
    
    with mss.mss() as sct:
        monitor = {"top": y, "left": x, "width": width, "height": height}
        sct_img = sct.grab(monitor)
        img = np.array(sct_img)
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    
    # Create a timestamped filename for the screenshot
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"screenshot_{timestamp}.png"
    
    # Create the full file path
    file_path = os.path.join(screenshot_folder, filename)
    
    # Save the screenshot
    cv2.imwrite(file_path, img)
    print(f"Screenshot saved to: {file_path}")

# Ensure the status FIFO exists
if not os.path.exists(statusa_fifo_path):
    os.mkfifo(statusa_fifo_path)

for fifo_path in [command_fifo_path, abort_fifo_path]:
    if not os.path.exists(fifo_path):
        os.mkfifo(fifo_path)

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and flies to aTargetAltitude.
    """
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        if stop_requested:
            return 'stop'
        print("Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        if stop_requested:
            return 'stop'
        print("Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        if stop_requested:
            return 'stop'
        current_altitude = vehicle.location.global_relative_frame.alt
        print(f"Altitude: {current_altitude}")
        if current_altitude >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def set_yaw(angle, direction, relative=False):
    """
    Sets the vehicle's yaw to a specific heading.
    """
    is_relative = 1 if relative else 0
    direction_val = 1 if direction == 'C' else -1

    msg = vehicle.message_factory.command_long_encode(
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        0,  # confirmation
        angle,  # param 1, yaw in degrees
        0,  # param 2, yaw speed (degrees/second, 0 uses default)
        direction_val,  # param 3, direction: 1=CW, -1=CCW
        is_relative,  # param 4, relative offset 1, absolute angle 0
        0, 0, 0  # param 5 ~ 7 not used
    )
    vehicle.send_mavlink(msg)
    
    # Wait for the yaw to complete
    start_time = time.time()
    while True:
        if stop_requested:
            return 'stop'
        # Yaw completion can be checked with the difference between desired and current heading
        current_heading = vehicle.heading
        target_heading = (current_heading + (angle if direction == 'C' else -angle)) % 360
        if abs(current_heading - target_heading) < 1:  # Threshold for heading accuracy
            break
        if time.time() - start_time > 4:  # Timeout for yaw action
            print("Yaw command timed out.")
            break
        time.sleep(0.5)

def calculate_target_coordinates(distance, bearing):
    R = 6378137.0  # Earth radius in meters
    distance_in_radians = distance / R

    current_lat = vehicle.location.global_relative_frame.lat
    current_lon = vehicle.location.global_relative_frame.lon

    lat1 = math.radians(current_lat)
    lon1 = math.radians(current_lon)
    bearing = math.radians(bearing)

    lat2 = math.asin(math.sin(lat1) * math.cos(distance_in_radians) +
                     math.cos(lat1) * math.sin(distance_in_radians) * math.cos(bearing))

    lon2 = lon1 + math.atan2(math.sin(bearing) * math.sin(distance_in_radians) * math.cos(lat1),
                             math.cos(distance_in_radians) - math.sin(lat1) * math.sin(lat2))

    new_lat = math.degrees(lat2)
    new_lon = math.degrees(lon2)

    return new_lat, new_lon

def fiforead(fifo_path):
    if not os.path.exists(fifo_path):
        os.mkfifo(fifo_path)
    
    # Open the FIFO in non-blocking mode
    try:
        with open(fifo_path, 'r', os.O_NONBLOCK) as fifo:
            return fifo.read()
    except Exception as e:
        print(f"Error reading from FIFO {fifo_path}: {e}")
        return ""

def fifowrite(fifo_path, whatToWrite):
    if not os.path.exists(fifo_path):
        os.mkfifo(fifo_path)
    
    try:
        with open(fifo_path, 'w', os.O_NONBLOCK) as fifo:
            fifo.write(whatToWrite)
    except Exception as e:
        print(f"Error writing to FIFO {fifo_path}: {e}")


def move_to_target(target_lat, target_lon, target_alt=None):
    if target_alt is not None:
        target_location = LocationGlobalRelative(target_lat, target_lon, target_alt)
    else:
        target_location = LocationGlobalRelative(target_lat, target_lon, vehicle.location.global_relative_frame.alt)
    vehicle.simple_goto(target_location)

    while not stop_requested:
        current_location = vehicle.location.global_relative_frame
        dist_to_target = get_distance_metres(current_location, target_location)
        
        if target_alt is not None:
            alt_diff = abs(vehicle.location.global_relative_frame.alt - target_alt)
        else:
            alt_diff = 0
        
        if dist_to_target <= 1.5 and alt_diff <= 0.8:
            print("Reached target location")
            break
        time.sleep(1)
    
    if stop_requested:
        stop_drone()
        return 'stop'

def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5

def set_altitude(altitude):
    result = move_to_target(vehicle.location.global_relative_frame.lat,
                            vehicle.location.global_relative_frame.lon,
                            altitude)
    return result

def update_status():

    global stop_requested
    while True:
        if stop_requested:
            return
        current_location = vehicle.location.global_relative_frame
        current_heading = vehicle.heading
        status = {
            "latitude": current_location.lat,
            "longitude": current_location.lon,
            "altitude": current_location.alt,
            "bearing": current_heading  
        }
        status_str = json.dumps(status) 
        with open(status_fifo_path, 'w') as fifo:
            fifo.write(status_str + '\n')
        with open(statusa_fifo_path, 'w') as fifo:
            fifo.write(status_str + '\n')
        time.sleep(0.2) 



      
def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # Use the body frame
        0b0000111111000111,  # Bitmask to indicate velocity components are enabled
        0, 0, 0,
        velocity_x, velocity_y, velocity_z,
        0, 0, 0, 0, 0)
    
    # Convert duration to seconds and ensure it runs the full duration even if fractional
    end_time = time.time() + duration
    while time.time() < end_time:
        vehicle.send_mavlink(msg)
        time.sleep(0.1)  # Send the message every 0.1 seconds


def move_circle_with_velocity(radius, portion, steps, velocity, quadrant, clockwise):
    """
    Move the drone in a circular path according to the given parameters using NED velocity.
    """
    angle_increment = (2 * math.pi * portion) / steps
    direction = 1 if clockwise else -1
    
    # Determine the starting angle based on quadrant and direction
    if quadrant == 4:
        start_angle = 0 if clockwise else math.pi / 2
    elif quadrant == 3:
        start_angle = math.pi / 2 if clockwise else math.pi
    elif quadrant == 2:
        start_angle = math.pi if clockwise else 3 * math.pi / 2
    elif quadrant == 1:
        start_angle = 3 * math.pi / 2 if clockwise else 0
    
    # Traverse the circle
    for step in range(steps):
        angle = start_angle + step * angle_increment * direction
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        
        # Calculate the velocities in the body frame
        vx = -velocity * math.sin(angle)
        vy = velocity * math.cos(angle)
        
        # Send the velocity command
        send_ned_velocity(vx, vy, 0, angle_increment / velocity)  # Adjust duration based on velocity and angle increment


      
def execute_command(command):
    global stop_requested

    if command == 'STOP':
        stop_drone()
        return 'stop'

    try:
        if command.startswith('W'):
            secs = float(command[1:])
            print(f"Waiting for {secs} seconds.")
            for _ in range(int(secs * 10)):  # Check for STOP every 0.1 second
                if stop_requested:
                    return 'stop'
                time.sleep(0.1)
            return True

        if command.startswith('T'):
            target_altitude = float(command[1:])
            return arm_and_takeoff(target_altitude)

        elif command.startswith('ALT'):
            altitude = float(command[3:])
            return set_altitude(altitude)

        elif command.startswith('GOTO'):
            coords = command[5:-1].split(',')
            target_lat = float(coords[0])
            target_lon = float(coords[1])
            return move_to_target(target_lat, target_lon)
        
        elif command.startswith('SEE'):
            # Handle the SEE command by extracting the full context
            see_context_start = command.find('(') + 1
            see_context_end = command.rfind(')')  # Use rfind to find the last closing parenthesis
            see_context = command[see_context_start:see_context_end].strip()

            take_screenshot()
            fifowrite(imgcont_fifo_path, "i")
            fifowrite(imgcontnew_fifo_path, "i")
            time.sleep(0.1)
            fifowrite(seecont_fifo_path, see_context)
            print(f"SEE command processed with context: {see_context}")
            time.sleep(7)
            return True
                    
            
            
        
        elif command.startswith('CIRC'):
            # Extract the parameters from the CIRC command
            params = command[5:-1].split(',')
            radius = int(params[0])
            portion = float(params[1])
            quadrant = int(params[2])
            clockwise = params[3].strip().lower() == 'true'
            
            return move_circle_with_velocity(radius, portion, 10, 1, quadrant, clockwise)

        elif command[0] in ['A', 'C']:
            direction = command[0]
            angle = float(command[1:])
            return set_yaw(angle, direction, relative=True)

        elif command == 'LAND':
            vehicle.mode = VehicleMode("LAND")
            return True

        elif command == 'RTL':
            vehicle.mode = VehicleMode("RTL")
            return True

        elif command[0] in ['F', 'B', 'L', 'R']:
            distance = float(command[1:])
            current_heading = vehicle.heading
            vehicle.parameters['WP_YAW_BEHAVIOR'] = 0

            if command[0] == 'F':
                target_lat, target_lon = calculate_target_coordinates(distance, current_heading)
            elif command[0] == 'B':
                target_lat, target_lon = calculate_target_coordinates(distance, current_heading + 180)
            elif command[0] == 'L':
                target_lat, target_lon = calculate_target_coordinates(distance, current_heading - 90)
            elif command[0] == 'R':
                target_lat, target_lon = calculate_target_coordinates(distance, current_heading + 90)
            else:
                print("Invalid command")
                return False

            return move_to_target(target_lat, target_lon)

        else:
            print("Invalid command.")
            return False

    except ValueError:
        print("Invalid command format. Please use the correct format.")
        return False

    return True  # Signal to continue processing further commands

def stop_drone():
    """
    Stops the drone using zero velocity in GUIDED mode.
    """
    print("Stopping drone by setting zero velocity in GUIDED mode.")
    set_velocity_body(vehicle, 0, 0, 0)
    global stop_requested
    stop_requested = True

def set_velocity_body(vehicle, vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0)
    vehicle.send_mavlink(msg)

def monitor_abort_fifo():
    global stop_requested
    if not os.path.exists(abort_fifo_path):
        os.mkfifo(abort_fifo_path)
    while True:
        with open(abort_fifo_path, 'r') as fifo:
            command = fifo.read().strip()
            if command.lower() == 'stop':
                stop_requested = True
                stop_drone()

def main():
    global stop_requested

    threading.Thread(target=update_status, daemon=True).start()
    threading.Thread(target=monitor_abort_fifo, daemon=True).start()
    
    while True:
        stop_requested = False  # Reset stop_requested before each new command set
        with open(command_fifo_path, 'r') as fifo:
            # Read a line (message) from the FIFO
            command_string = fifo.readline().strip()
        commands = command_string.split()
        for command in commands:
            if stop_requested:
                break  # Exit the current loop if stop is requested
            print(f"Executing command: {command}")
            result = execute_command(command)
            if result == 'stop':
                break

if __name__ == "__main__":
    main()



'''

qhyhu jrqqd jlyh x xs
qhyhu jrqqd ohw brx grzq
qhyhu jrqqd uxq durxqg dqg

ghvvhuw brx

'''
