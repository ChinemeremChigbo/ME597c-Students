import sys

from utilities import Logger, euler_from_quaternion
from rclpy.time import Time
from rclpy.node import Node

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Odometry as odom

from rclpy import init, spin, spin_once

rawSensor = 0
class localization(Node):
    
    def __init__(self, localizationType=rawSensor):

        super().__init__("localizer")
        
        # TODO Part 3: Define the QoS profile variable based on whether you are using the simulation (Turtlebot 3 Burger) or the real robot (Turtlebot 4)
        # Remember to define your QoS profile based on the information available in "ros2 topic info /odom --verbose" as explained in Tutorial 3
        odom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.loc_logger=Logger("robot_pose.csv", ["x", "y", "theta", "stamp"])
        self.pose=None
        
        if localizationType == rawSensor:
        # TODO Part 3: subscribe to the position sensor topic (Odometry)
            self.create_subscription(odom, "/odom", self.odom_callback, odom_qos)
        else:
            print("This type doesn't exist", file=sys.stderr)
    
    def odom_callback(self, pose_msg):
        # TODO Part 3: Read x, y, theta, and record the stamp
        x = pose_msg.pose.pose.position.x
        y = pose_msg.pose.pose.position.y
        
        # Convert quaternion to yaw (theta)
        orientation_q = pose_msg.pose.pose.orientation
        theta = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        # Record timestamp
        timestamp = pose_msg.header.stamp

        # Save pose
        self.pose = [x, y, theta, timestamp]
        print(f"Updated Pose: {self.pose}")

        # Log the data
        self.loc_logger.log_values([x, y, theta, Time.from_msg(timestamp).nanoseconds])
    
    def getPose(self):
        return self.pose

# TODO Part 3: Run only if executed directly
if __name__ == "__main__":
    init()  # Initialize ROS2
    node = localization()
    spin(node)  # Keep the node running
    node.destroy_node()
