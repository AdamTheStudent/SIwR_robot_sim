#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from threading import Thread
import time
import utm
import gtsam
from gtsam import symbol_shorthand as sym
import numpy as np

# Global variables to store sensor data
gps_data = None
odom_data = None

gps_x = []
gps_y = []
odom_x = []
odom_y = []

# Global variables for the factor graph
initial_utm_x = None
initial_utm_y = None
pose_index = 0
graph = gtsam.NonlinearFactorGraph()
initial_estimate = gtsam.Values()
estimate_x = []
estimate_y = []

class SensorListener(Node):

    def __init__(self):
        super().__init__('sensor_listener')
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10)
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10)

    def gps_callback(self, msg):
        global gps_data
        gps_data = msg

    def odom_callback(self, msg):
        global odom_data
        odom_data = msg

def update_data():
    global gps_data, odom_data, initial_utm_x, initial_utm_y, pose_index

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

            # Add GPS factor to the graph
            gps_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.5, 0.5, 0.1]))  # noise model
            graph.add(gtsam.PriorFactorPose2(sym.X(pose_index), gtsam.Pose2(local_utm_x, local_utm_y, 0), gps_noise))

        if odom_data:
            # Get odometry data
            local_odom_x = odom_data.pose.pose.position.x
            local_odom_y = odom_data.pose.pose.position.y

            odom_x.append(local_odom_x)
            odom_y.append(local_odom_y)

            # Add odometry factor between consecutive poses
            if pose_index > 0:
                odom_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.1]))  # noise model
                odometry = gtsam.Pose2(local_odom_x - odom_x[-2], local_odom_y - odom_y[-2], 0)
                graph.add(gtsam.BetweenFactorPose2(sym.X(pose_index - 1), sym.X(pose_index), odometry, odom_noise))

            # Initialize the estimate for this pose
            initial_estimate.insert(sym.X(pose_index), gtsam.Pose2(local_odom_x, local_odom_y, 0))
            pose_index += 1

        time.sleep(0.1)  # Update rate

def optimize_and_plot_result():
    global pose_index

    while rclpy.ok():  # Continuously optimize and update the plot
        if pose_index > 0:
            optimizer = gtsam.GaussNewtonOptimizer(graph, initial_estimate)
            result = optimizer.optimize()

            # Clear previous estimates
            estimate_x.clear()
            estimate_y.clear()

            # Extract and plot the estimated positions
            for i in range(pose_index):
                pose = result.atPose2(sym.X(i))
                estimate_x.append(pose.x())
                estimate_y.append(pose.y())

        time.sleep(1)  # Optimize and update every second

def animate(i):
    plt.clf()
    plt.subplot(3, 1, 1)
    plt.plot(gps_x, gps_y, 'r-', label="GPS")
    plt.title('GPS Data (UTM)')
    plt.xlabel('Easting (m)')
    plt.ylabel('Northing (m)')
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(odom_x, odom_y, 'b-', label="Odometry")
    plt.title('Odometry Data')
    plt.xlabel('Position X')
    plt.ylabel('Position Y')
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(estimate_x, estimate_y, 'k-', label="Estimated Position")
    plt.title('Estimated Position using Factor Graph')
    plt.xlabel('Position X')
    plt.ylabel('Position Y')
    plt.legend()

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

    # Start optimization and plotting in a separate thread
    opt_thread = Thread(target=optimize_and_plot_result)
    opt_thread.start()
    
    # Start plotting
    start_plot()

    sensor_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
