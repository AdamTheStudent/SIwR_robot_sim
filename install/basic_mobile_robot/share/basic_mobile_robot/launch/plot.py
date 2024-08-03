import rospy
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from threading import Thread
import time

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

def gps_callback(data):
    global gps_data
    gps_data = data

def imu_callback(data):
    global imu_data
    imu_data = data

def odom_callback(data):
    global odom_data
    odom_data = data

def listener():
    rospy.init_node('sensor_listener', anonymous=True)
    rospy.Subscriber("/gps_topic", NavSatFix, gps_callback)
    rospy.Subscriber("/imu_topic", Imu, imu_callback)
    rospy.Subscriber("/odom_topic", Odometry, odom_callback)
    rospy.spin()

def update_data():
    global gps_data, imu_data, odom_data
    while not rospy.is_shutdown():
        if gps_data:
            gps_x.append(gps_data.longitude)
            gps_y.append(gps_data.latitude)
        if imu_data:
            imu_x.append(imu_data.linear_acceleration.x)
            imu_y.append(imu_data.linear_acceleration.y)
        if odom_data:
            odom_x.append(odom_data.pose.pose.position.x)
            odom_y.append(odom_data.pose.pose.position.y)
        time.sleep(0.1)  # Update rate

def animate(i):
    plt.clf()
    plt.subplot(3, 1, 1)
    plt.plot(gps_x, gps_y, 'r-')
    plt.title('GPS Data')
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.subplot(3, 1, 2)
    plt.plot(imu_x, imu_y, 'g-')
    plt.title('IMU Data')
    plt.xlabel('Linear Acceleration X')
    plt.ylabel('Linear Acceleration Y')
    plt.subplot(3, 1, 3)
    plt.plot(odom_x, odom_y, 'b-')
    plt.title('Odometry Data')
    plt.xlabel('Position X')
    plt.ylabel('Position Y')
    plt.tight_layout()

def start_plot():
    ani = animation.FuncAnimation(plt.gcf(), animate, interval=1000)
    plt.show()

if __name__ == '__main__':
    # Start ROS listener thread
    ros_thread = Thread(target=listener)
    ros_thread.start()
    
    # Start data update thread
    data_thread = Thread(target=update_data)
    data_thread.start()
    
    # Start plotting
    start_plot()

