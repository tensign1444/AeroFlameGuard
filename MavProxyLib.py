import math
import time
#import picamera
import os
import platform
import sys
from dronekit import connect, VehicleMode,LocationGlobal,LocationGlobalRelative
from pymavlink import mavutil


class Drone:
    def __init__(self, manualArm, targetAltitude):
        """
        Initialize the Drone instance.

        Args:
            manualArm: A boolean flag indicating whether the drone should be armed manually.
            targetAltitude: The desired altitude of the drone in meters.

        Returns:
            None.
        """
        #IP Address of drone on CUI_WPA2 10.20.91.166
        self.vehicle = None
        print "Attempting to connect to drone "
        self.vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)
        print "Battery is at " + str(self.vehicle.battery)
        self.vehicle.mode = VehicleMode("GUIDED")
        count = 0
        while self.vehicle.mode != 'GUIDED' and count < 30:
                print "Waiting for drone to enter GUIDED flight mode... %s" % (count)
                time.sleep(1)
                count += 1
        if count >= 30:
                self.__del__()
        print "Vehicle now in GUIDED MODE"


    def __del__(self):
        if self.vehicle:
            if self.vehicle.mode != 'LAND':
                self.land()
            if not self.vehicle.armed:
                self.disarm()
            self.vehicle.close()


    def get_Lat_Lon(self):
        i = 0
        lat_sum = 0
        lon_sum = 0
        avg_lat = 0
        avg_lon = 0
        for i in range(10):
            # Get the current latitude and longitude
            lat = self.vehicle.location.global_frame.lat
            lon = self.vehicle.location.global_frame.lon

            # Accumulate the coordinates
            lat_sum += lat
            lon_sum += lon

            # Wait for a short time before taking the next reading
            time.sleep(0.1)

            # Compute the average coordinates
        avg_lat = lat_sum / 10
        avg_lon = lon_sum / 10

        print "%.6f, %.6f" % (avg_lat, avg_lon)



    def arm(self):
        print "Attempting to arm drone"
        count = 0
        while not self.vehicle.is_armable and count < 15:
            print "Waiting for vehicle to initialize... %s" % (count)
            time.sleep(1)
            count += 1
        if not self.vehicle.is_armable:
            return self.vehicle.armed
        print "Vehicle is now armable"
        self.vehicle.armed = True
        count = 0
        while not self.vehicle.armed and count < 5:
            print "Waiting for vehicle to arm itself... %s" % (count)
            time.sleep(1)
            count += 1
        print "Drone is now ready"
        return self.vehicle.armed

    def takeoff(self, takeoff_height=1):
        self.arm()
        print "Attempting to takeoff drone"
        height = 0
        count = 0
        if height > 10:
            print "Capping takeoff height from %sm to 10m" % (takeoff_height)
            takeoff_height = 10
        self.vehicle.simple_takeoff(takeoff_height)
        timeout = takeoff_height * 5
        while height < 0.95 * takeoff_height and count < timeout:
            height = self.vehicle.location.global_relative_frame.alt
            print "Drone is now at height %sm... %s" % (height, count)
            time.sleep(1)
            count += 1
        return height

    def land(self):
        """
        Command the drone to land by setting its mode to LAND.

        Returns:
            None.
        """
        self.vehicle.mode = VehicleMode("LAND")
        while self.vehicle.mode != 'LAND':
            time.sleep(1)
            print "Waiting for drone to land"
        print "Drone in land mode. "
        self.inFlight = False
        self.disarm()



    def disarm(self):
        print "Attempting to disarm drone"
        if self.vehicle == None:
            raise
        self.vehicle.armed = False
        count = 0
        while self.vehicle.armed and count < 10:
            print "Waiting for vehicle to disarm itself... %s" % (count)
            time.sleep(1)
            count += 1
        return self.vehicle.armed

    def send_local_ned_velocity(self, vx, vy, vz):
        """
        Send a local NED velocity command to the drone.

        Args:
            vx: The desired velocity in the x direction in meters per second.
            vy: The desired velocity in the y direction in meters per second.
            vz: The desired velocity in the z direction in meters per second.

        Returns:
            None.
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000111111000111,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, 0)
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def send_global_ned_velocity(self, vx, vy, vz):
        """
        Send a global NED velocity command to the drone.

        Args:
            vx: The desired velocity in the x direction in meters per second.
            vy: The desired velocity in the y direction in meters per second.
            vz: The desired velocity in the z direction in meters per second.

        Returns:
            None.
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        vx, vy, vz,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def send_global_ned_velocity(self, vx, vy, vz):
        """
        Send a velocity command in the global NED frame.

        Args:
            vx: The velocity in the x direction (north) in meters per second.
            vy: The velocity in the y direction (east) in meters per second.
            vz: The velocity in the z direction (down) in meters per second.

        Returns:
            None.
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b0000111111000111,  # type_mask (only speeds enabled)
            0, 0, 0,  # x, y, z positions (not used)
            vx, vy, vz,  # x, y, z velocity in m/s
            0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def move_forward(self, x):
        """
        Move the drone forward.

        Args:
            x: The distance to move in the x direction (forward) in meters.

        Returns:
            None.
        """
        self.send_local_ned_velocity(x, 0, 0)
        print "Moving NORTH relative to front of drone..."

    def move_back(self, x):
        """
        Move the drone back.

        Args:
            x: The distance to move in the x direction (back) in meters.

        Returns:
            None.
        """
        self.send_local_ned_velocity(-x, 0, 0)
        print "Moving meters back relative to front of drone..."

    def move_left(self, y):
        """
        Move the drone left.

        Args:
            y: The distance to move in the y direction (left) in meters.

        Returns:
            None.
        """
        self.send_local_ned_velocity(0, -y, 0)
        print "Moving  meters left relative to front of drone..."

    def move_right(self, y):
        """
        Move the drone right.

        Args:
            y: The distance to move in the y direction (right) in meters.

        Returns:
            None.
        """
        self.send_local_ned_velocity(0, y, 0)
        print "Moving meters right relative to front of drone..."

    def move_up(self, z):
        """
        Move the drone up.

        Args:
            z: The distance to move in the z direction (up) in meters.

        Returns:
            None.
        """
        self.send_local_ned_velocity(0, 0, -z)
        print "Moving up..."

    def move_down(self, z):
        """
        Move the drone down.

        Args:
            z: The distance to move in the z direction (down) in meters.

        Returns:
            None.
        """
        self.send_local_ned_velocity(0, 0, z)
        print "Moving down..."

    def global_move_forward(self, x):
        """
        Move the drone forward in the global NED frame.

        Args:
            x: The distance to move in the x direction (forward) in meters.

        Returns:
            None.
        """
        self.send_global_ned_velocity(x, 0, 0)
        print "Moving" + x + "  meters NORTH relative to front of drone..."

    def global_move_back(self, x):
        """
        Move the drone back in the global NED frame.

        Args:
            x: The distance to move in the x direction (back) in meters.

        Returns:
            None.
        """
        self.send_global_ned_velocity(-x, 0, 0)
        print "Moving" + x + "  meters back relative to front of drone..."

    def global_move_left(self, y):
        """
        Move the drone left in the global NED frame.

        Args:
            y: The distance to move in the y direction (left) in meters.

        Returns:
            None.
        """
        self.send_global_ned_velocity(0, -y, 0)
        print "Moving" + y + "  meters left relative to front of drone..."

    def global_move_right(self, y):
        """
        Move the drone right in the global NED frame.

        Args:
            y: The distance to move in the y direction (right) in meters.

        Returns:
            None.
        """
        self.send_global_ned_velocity(0, y, 0)
        print "Moving" + y + "  meters right relative to front of drone..."

    def global_move_up(self, z):
        """
        Move the drone upwards by a specified distance in meters using global NED velocity.

        Args:
            z: The desired distance to move the drone upwards in meters.

        Returns:
            None.
        """
        self.send_global_ned_velocity(0, 0, -z)
        print "Moving" + z + "  meters up..."

    def global_move_down(self, z):
        """
        Move the drone downwards by a specified distance in meters using global NED velocity.

        Args:
            z: The desired distance to move the drone downwards in meters.

        Returns:
            None.
        """
        self.send_global_ned_velocity(0, 0, z)
        print "Moving" + z + "  meters down..."

    def navigate_to_lat_lon(self, lat, lon, height):
        """
        Navigate the drone to a specified latitude, longitude, and altitude using the simple_goto() method.

        Args:
            lat: The desired latitude of the drone in decimal degrees.
            lon: The desired longitude of the drone in decimal degrees.
            height: The desired altitude of the drone in meters.

        Returns:
            None.
        """
        a_location = LocationGlobalRelative(lat, lon, height)
        self.vehicle.simple_goto(a_location)
        print "Moving..."

    def condition_yaw(self, heading, relative=False):
        """
        Rotate the drone to a specified heading using the CONDITION_YAW MAVLink command.

        Args:
            heading: The desired heading of the drone in degrees.
            relative: A flag indicating whether the heading is relative to the drone's current heading.

        Returns:
            None.
        """
        if relative:
            is_relative = 1  # yaw relative to direction of travel
        else:
            is_relative = 0  # yaw is an absolute angle
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
            0,  # confirmation
            heading,  # param 1, yaw in degrees
            0,  # param 2, yaw speed deg/s
            1,  # param 3, direction -1 ccw, 1 cw
            is_relative,  # param 4, relative offset 1, absolute angle 0
            0, 0, 0)  # param 5 ~ 7 not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

    def rotate_CCW(self, degrees):
        """
        Rotate the drone counterclockwise by a given number of degrees.

        Args:
            degrees: The number of degrees to rotate the drone counterclockwise.

        Returns:
            None.
        """
        print "Rotating CCW.."
        yaw_angle = math.radians(degrees)
        self.vehicle.simple_yaw_rate_control(yaw_rate)

    def rotate_CW(self, degrees):
        """
        Rotate the drone clockwise by a given number of degrees.

        Args:
            degrees: The number of degrees to rotate the drone clockwise.

        Returns:
            None.
        """
        yaw_angle = math.radians(degrees)

        msg = self.vehicle.message_factory.set_attitude_target_encode(
            0,  # time_boot_ms
            1,  # target system
            1,  # target component
            mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame
            0b00000000,  # type_mask - ignore all fields except yaw angle
            0,  # no roll angle
            0,  # no pitch angle
            -yaw_angle,  # yaw angle (negative for clockwise rotation)
            0,  # no roll rate
            0,  # no pitch rate
            0,  # no yaw rate
            0,  # no throttle
        )

        self.vehicle.send_mavlink(msg)

    def move_arc(self, arc_angle, radius, speed, pitch_angle, roll_angle):
        """
        Move the drone forward along an arc with a given arc angle and radius.

        Args:
            vehicle: The dronekit Vehicle object representing the connected drone.
            arc_angle: The arc angle to move along, in degrees.
            radius: The radius of the arc, in meters.
            speed: The speed of the drone, in meters per second.
            pitch_angle: The pitch angle of the drone, in degrees.
            roll_angle: The roll angle of the drone, in degrees.

        Returns:
            None.
        """
        # Convert pitch and roll angles from degrees to radians
        pitch_angle = math.radians(pitch_angle)
        roll_angle = math.radians(roll_angle)

        # Calculate the yaw rate to maintain the same heading
        yaw_rate = speed / radius

        # Calculate the time it takes to travel 1 meter
        time_per_meter = 1 / speed

        # Calculate the distance traveled along the arc for each iteration
        distance_per_iteration = radius * math.radians(1)

        # Calculate the total distance to travel along the arc
        total_distance = radius * math.radians(arc_angle)

        # Initialize the current distance traveled along the arc to 0
        current_distance = 0

        # Move the drone along the arc
        while current_distance < total_distance:
            # Calculate the current pitch angle based on the distance traveled
            current_pitch_angle = math.atan(current_distance / radius)

            # Create a message to set the attitude of the drone
            msg = self.vehicle.message_factory.set_attitude_target_encode(
                0,  # time_boot_ms
                1,  # target system
                1,  # target component
                0b00000110,  # type_mask - ignore all fields except pitch and roll
                0,  # no yaw angle
                pitch_angle + current_pitch_angle,  # pitch angle
                roll_angle,  # roll angle
                yaw_rate,  # yaw rate
                0,  # no pitch rate
                0,  # no roll rate
                0,  # no yaw rate
                0.5,  # set throttle to 50%
            )

            # Send the message
            self.vehicle.send_mavlink(msg)

            # Wait for the drone to travel 1 meter
            time.sleep(time_per_meter)

            # Update the current distance traveled along the arc
            current_distance += distance_per_iteration

    def check_battery(self):
        """
        Check the battery percentage of the connected drone.

        Args:
            vehicle: The dronekit Vehicle object representing the connected drone.

        Returns:
            The battery percentage of the drone as a float.
        """
        battery = self.vehicle.battery
        return battery.level


    def do_flip(self):
        """
        Perform a flip while hovering the drone.

        Args:
            vehicle: The dronekit Vehicle object representing the connected and armed drone.

        Returns:
            None.
        """
        roll_angle = math.radians(180)

        pitch_angle = 0
        yaw_rate = 0

        throttle = 0.5

        msg = self.vehicle.message_factory.set_attitude_target_encode(
            0,  # time_boot_ms
            1,  # target system
            1,  # target component
            mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame
            0b00000111,  # type_mask - ignore all fields except roll angle
            roll_angle,  # roll angle
            pitch_angle,  # pitch angle
            0,  # no yaw angle
            0,  # no roll rate
            0,  # no pitch rate
            yaw_rate,  # yaw rate
            throttle,  # throttle
        )


        self.vehicle.send_mavlink(msg)

        time.sleep(2)

        throttle = 0

        msg = self.vehicle.message_factory.set_attitude_target_encode(
            0,  # time_boot_ms
            1,  # target system
            1,  # target component
            mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame
            0b00001111,  # type_mask - ignore no fields
            0,  # no roll angle
            0,  # no pitch angle
            0,  # no yaw angle
            0,  # no roll rate
            0,  # no pitch rate
            0,  # no yaw rate
            throttle,  # throttle
        )

        self.vehicle.send_mavlink(msg)

        time.sleep(1)

"""        def record_video(filename):
            # Initialize the camera
            camera = picamera.PiCamera()

            # Set camera resolution
            camera.resolution = (640, 480)

            # Set video format and framerate
            camera.framerate = 24
            camera.video_format = 'h264'

            # Start recording
            camera.start_recording(filename)

            while(True):
                if(not self.inFlight):
                    # Stop recording and close the camera
                    camera.stop_recording()
                    camera.close()
                    break
"""
class Util():

    def get_distance_metres_global(aLocation1, aLocation2):
        """
        Returns the ground distance in metres between two `LocationGlobal` or `LocationGlobalRelative` objects.

        This method is an approximation, and will not be accurate over large distances and close to the
        earth's poles. It comes from the ArduPilot test code:
        https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
        """
        dlat = aLocation2.lat - aLocation1.lat
        dlong = aLocation2.lon - aLocation1.lon
        return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5

    def get_location_metres(original_location, dNorth, dEast):
        """
        Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
        specified `original_location`. The returned LocationGlobal has the same `alt` value
        as `original_location`.

        The function is useful when you want to move the vehicle around specifying locations relative to
        the current vehicle position.

        The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

        For more information see:
        http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
        """
        earth_radius = 6378137.0  # Radius of "spherical" earth
        # Coordinate offsets in radians
        dLat = dNorth / earth_radius
        dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))

        # New position in decimal degrees
        newlat = original_location.lat + (dLat * 180 / math.pi)
        newlon = original_location.lon + (dLon * 180 / math.pi)
        if type(original_location) is LocationGlobal:
            targetlocation = LocationGlobal(newlat, newlon, original_location.alt)
        elif type(original_location) is LocationGlobalRelative:
            targetlocation = LocationGlobalRelative(newlat, newlon, original_location.alt)
        else:
            raise Exception("Invalid Location object passed")

        return targetlocation;

    def get_bearing(aLocation1, aLocation2):
        """
        Returns the bearing between the two LocationGlobal objects passed as parameters.

        This method is an approximation, and may not be accurate over large distances and close to the
        earth's poles. It comes from the ArduPilot test code:
        https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
        """
        off_x = aLocation2.lon - aLocation1.lon
        off_y = aLocation2.lat - aLocation1.lat
        bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
        if bearing < 0:
            bearing += 360.00
        return bearing;
