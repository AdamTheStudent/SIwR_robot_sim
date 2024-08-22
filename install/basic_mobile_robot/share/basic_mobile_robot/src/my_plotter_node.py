# my_plotter_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class PlotterNode(Node):
    def __init__(self):
        super().__init__('plotter_node')
        self.sub1 = self.create_subscription(Float32, '/fix/gps', self.listener_callback1, 10)
        self.sub2 = self.create_subscription(Float32, '/wheel/odometry', self.listener_callback2, 10)
        self.sub3 = self.create_subscription(Float32, '/wheel/odometry', self.listener_callback3, 10)
        
        self.data1 = []
        self.data2 = []
        self.data3 = []
        
        # Setup the figure and axis
        self.fig, self.ax = plt.subplots()
        self.line1, = self.ax.plot([], [], label='Topic 1')
        self.line2, = self.ax.plot([], [], label='Topic 2')
        self.line3, = self.ax.plot([], [], label='Topic 3')
        
        plt.legend()

    def listener_callback1(self, msg):
        self.data1.append(msg.data)
    
    def listener_callback2(self, msg):
        self.data2.append(msg.data)
    
    def listener_callback3(self, msg):
        self.data3.append(msg.data)

    def update_plot(self, frame):
        self.line1.set_data(range(len(self.data1)), self.data1)
        self.line2.set_data(range(len(self.data2)), self.data2)
        self.line3.set_data(range(len(self.data3)), self.data3)
        
        self.ax.relim()
        self.ax.autoscale_view()
        return self.line1, self.line2, self.line3

    def start_plotting(self):
        ani = FuncAnimation(self.fig, self.update_plot, blit=True)
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    plotter_node = PlotterNode()

    import threading
    plot_thread = threading.Thread(target=plotter_node.start_plotting)
    plot_thread.start()
    
    rclpy.spin(plotter_node)
    
    plotter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

