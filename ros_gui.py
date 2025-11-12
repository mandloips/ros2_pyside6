# ros_gui.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import random
from data_logger import log_pose_data  # Import the data logging module

class RosGui(Node):
    #setting up live variables along with publisher and subscriber
    def __init__(self):
        super().__init__("ros_gui")

        self.x = None
        self.y = None
        self.theta = None
        self.linear_velocity = None
        self.angular_velocity = None

        self.cmv_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)

    #collecting pose data
    def pose_callback(self, msg: Pose):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
        self.linear_velocity = msg.linear_velocity
        self.angular_velocity = msg.angular_velocity
        log_pose_data(msg.x, msg.y, msg.theta, msg.linear_velocity, msg.angular_velocity)

    #start button action - randomly move the turtlebot
    def start_ros_timer(self):
        self.timer_ = self.create_timer(0.5, self.send_velocity_command)
        self.get_logger().info("Starting ROS timer...")

    #stop button action
    def stop_ros_timer(self):
        if hasattr(self, 'timer_'):
            self.timer_.cancel()
            self.get_logger().info("Stopped ROS timer.")
    
    #randomly move the turtlebot
    def send_velocity_command(self):
        msg = Twist()
        msg.linear.x = random.uniform(-2, 2)
        msg.angular.z = random.uniform(-1, 1)
        self.cmv_vel_pub_.publish(msg)
