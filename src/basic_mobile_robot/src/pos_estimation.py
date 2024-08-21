#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from threading import Thread
import time
import utm

# Global variables to store sensor data
gps_data = None
imu_data = None
odom_data = None

gps_x = []
gps_y = []
imu_x = []
imu_y = []
odom_x = []
odom_y = []

# Global variables to store the initial GPS coordinates
initial_utm_x = None
initial_utm_y = None

class SensorListener(Node):

    def __init__(self):
        super().__init__('sensor_listener')
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10)
        # self.imu_subscription = self.create_subscription(
        #     Imu,
        #     '/imu/data',
        #     self.imu_callback,
        #     10)
        # Filtr kalmana IMU + Odometria
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10)

    def gps_callback(self, msg):
        global gps_data
        gps_data = msg

    # def imu_callback(self, msg):
    #     global imu_data
    #     imu_data = msg

    def odom_callback(self, msg):
        global odom_data
        odom_data = msg

def update_data():
    global gps_data, imu_data, odom_data, initial_utm_x, initial_utm_y
    while rclpy.ok():
        if gps_data:
            # Convert latitude and longitude to UTM
            utm_coords = utm.from_latlon(gps_data.latitude, gps_data.longitude)
            
            # Set initial UTM coordinates if not already set
            if initial_utm_x is None and initial_utm_y is None:
                initial_utm_x = utm_coords[0]
                initial_utm_y = utm_coords[1]

            # Calculate the local UTM coordinates relative to the initial position
            local_utm_x = utm_coords[0] - initial_utm_x
            local_utm_y = utm_coords[1] - initial_utm_y

            gps_x.append(local_utm_x)
            gps_y.append(local_utm_y)

        # if imu_data:
        #     imu_x.append(imu_data.linear_acceleration.x)
        #     imu_y.append(imu_data.linear_acceleration.y)

        if odom_data:
            odom_x.append(odom_data.pose.pose.position.x)
            odom_y.append(odom_data.pose.pose.position.y)

        time.sleep(0.1)  # Update rate

def animate(i):
    plt.clf()
    plt.subplot(2, 1, 1)
    plt.plot(gps_x, gps_y, 'r-')
    plt.title('GPS Data (UTM)')
    plt.xlabel('Easting (m)')
    plt.ylabel('Northing (m)')
    # plt.subplot(3, 1, 2)
    # plt.plot(imu_x, imu_y, 'g-')
    # plt.title('IMU Data')
    # plt.xlabel('Linear Acceleration X')
    # plt.ylabel('Linear Acceleration Y')
    plt.subplot(2, 1, 2)
    plt.plot(odom_x, odom_y, 'b-')
    plt.title('Odometry Data')
    plt.xlabel('Position X')
    plt.ylabel('Position Y')
    plt.tight_layout()

def start_plot():
    ani = animation.FuncAnimation(plt.gcf(), animate, interval=1000)
    plt.show()

def main(args=None):
    rclpy.init(args=args)
    sensor_listener = SensorListener()

    # Start ROS listener thread
    ros_thread = Thread(target=rclpy.spin, args=(sensor_listener,))
    ros_thread.start()
    
    # Start data update thread
    data_thread = Thread(target=update_data)
    data_thread.start()
    
    # Start plotting
    start_plot()

    sensor_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()