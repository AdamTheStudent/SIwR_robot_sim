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

gps_data = None
odom_data = None

gps_x = []
gps_y = []
odom_x = []
odom_y = []

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
            utm_coords = utm.from_latlon(gps_data.latitude, gps_data.longitude)
            
            if initial_utm_x is None and initial_utm_y is None:
                initial_utm_x = utm_coords[0]
                initial_utm_y = utm_coords[1]
            local_utm_x = utm_coords[0] - initial_utm_x
            local_utm_y = utm_coords[1] - initial_utm_y

            gps_x.append(local_utm_x)
            gps_y.append(local_utm_y)
            gps_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.9, 0.9, 0.1]))
            graph.add(gtsam.PriorFactorPose2(sym.X(pose_index), gtsam.Pose2(local_utm_x, local_utm_y, 0), gps_noise))

        if odom_data:
            local_odom_x = odom_data.pose.pose.position.x
            local_odom_y = odom_data.pose.pose.position.y
            odom_x.append(local_odom_x)
            odom_y.append(local_odom_y)

            if pose_index > 0:
                odom_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.75, 0.75, 0.1]))
                odometry = gtsam.Pose2(local_odom_x - odom_x[-2], local_odom_y - odom_y[-2], 0)
                graph.add(gtsam.BetweenFactorPose2(sym.X(pose_index - 1), sym.X(pose_index), odometry, odom_noise))
            initial_estimate.insert(sym.X(pose_index), gtsam.Pose2(local_odom_x, local_odom_y, 0))
            pose_index += 1

        time.sleep(0.1)

def optimize_and_plot_result():
    global pose_index

    while rclpy.ok():  
        if pose_index > 0:
            optimizer = gtsam.GaussNewtonOptimizer(graph, initial_estimate)
            result = optimizer.optimize()

            estimate_x.clear()
            estimate_y.clear()

            for i in range(pose_index):
                key = sym.X(i)
                if result.exists(key): 
                    pose = result.atPose2(key)
                    estimate_x.append(pose.x())
                    estimate_y.append(pose.y())
                else:
                    print(f"Miejmy nadzieje ze tego nie widzisz")


        time.sleep(1)

def animate(i):
    plt.clf()
    plt.plot(odom_x, odom_y, 'b-', label="Odometry")
    plt.plot(estimate_x, estimate_y, 'k-', label="Estimated Position")
    plt.scatter(gps_x, gps_y, color='r', label="GPS", marker='o')
    
    plt.title('Sensor Data and Estimated Position')
    plt.xlabel('Position X')
    plt.ylabel('Position Y')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

def start_plot():
    ani = animation.FuncAnimation(plt.gcf(), animate, interval=1000)
    plt.show()

def main(args=None):
    rclpy.init(args=args)
    sensor_listener = SensorListener()
    ros_thread = Thread(target=rclpy.spin, args=(sensor_listener,))
    ros_thread.start()
    data_thread = Thread(target=update_data)
    data_thread.start()
    opt_thread = Thread(target=optimize_and_plot_result)
    opt_thread.start()
    
    start_plot()

    sensor_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
