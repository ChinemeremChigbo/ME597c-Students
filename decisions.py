# Imports


import sys

from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error
from pid import PID_ctrl

from rclpy import init, spin, spin_once
from rclpy.node import Node
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry as odom

from localization import localization, rawSensor

from planner import TRAJECTORY_PLANNER, POINT_PLANNER, planner
from controller import controller, trajectoryController

from math import sqrt

# You may add any other imports you may need/want to use below
# import ...


class decision_maker(Node):
    
    def __init__(self, publisher_msg, publishing_topic, qos_publisher, goalPoint, rate=10, motion_type=POINT_PLANNER, trajectory_type="parabola"):

        super().__init__("decision_maker")

        #TODO Part 4: Create a publisher for the topic responsible for robot's motion
        self.publisher = self.create_publisher(Twist, publishing_topic, qos_publisher)

        publishing_period=1/rate
        
        # Instantiate the controller
        # TODO Part 5: Tune your parameters here
    
        if motion_type == POINT_PLANNER:
            self.controller=controller(klp=0.2, klv=0.5, kap=0.8, kav=0.6)
            self.planner=planner(POINT_PLANNER)    
    
    
        elif motion_type==TRAJECTORY_PLANNER:
            self.controller=trajectoryController(klp=0.2, klv=0.5, kap=0.8, kav=0.6)
            self.planner = planner(TRAJECTORY_PLANNER, trajectory_type)

        else:
            print("Error! you don't have this planner", file=sys.stderr)


        # Instantiate the localization, use rawSensor for now  
        self.localizer=localization(rawSensor)

        # Instantiate the planner
        # NOTE: goalPoint is used only for the pointPlanner
        self.goal=self.planner.plan(goalPoint)

        self.create_timer(publishing_period, self.timerCallback)


    def timerCallback(self):
        
        # TODO Part 3: Run the localization node
        ...    # Remember that this file is already running the decision_maker node.
        spin_once(self.localizer)

        if self.localizer.getPose() is None:
            print("waiting for odom msgs ....")
            return

        # Compute distance to goal
        goal_threshold = 0.05
        current_pose = self.localizer.getPose()

        # TODO Part 3: Check if you reached the goal
        if isinstance(self.goal, list):  
            distance = sqrt((current_pose[0] - self.goal[0])**2 + (current_pose[1] - self.goal[1])**2)
            reached_goal = distance < goal_threshold
        else:
            reached_goal = False 

        if reached_goal:
            print("reached goal")

            vel_msg = Twist()
            self.publisher.publish(vel_msg) 
            
            self.controller.PID_angular.logger.save_log()
            self.controller.PID_linear.logger.save_log()
            
            #TODO Part 3: exit the spin
            self.destroy_node()
            sys.exit(0)
            return
        
        velocity, yaw_rate = self.controller.vel_request(self.localizer.getPose(), self.goal, True)

        #TODO Part 4: Publish the velocity to move the robot
        velocity, yaw_rate = self.controller.vel_request(self.localizer.getPose(), self.goal, True)

        vel_msg = Twist()
        vel_msg.linear.x = velocity
        vel_msg.angular.z = yaw_rate

        self.publisher.publish(vel_msg)

import argparse


def main(args=None):
    
    init()

    # TODO Part 3: You migh need to change the QoS profile based on whether you're using the real robot or in simulation.
    # Remember to define your QoS profile based on the information available in "ros2 topic info /odom --verbose" as explained in Tutorial 3
    
    odom_qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE
    )    

    # TODO Part 4: instantiate the decision_maker with the proper parameters for moving the robot
    if args.motion.lower() == "point":
        DM = decision_maker(publisher_msg=None, publishing_topic="/cmd_vel", qos_publisher=odom_qos, goalPoint=[1.0, 1.0])
    elif args.motion.lower() == "trajectory":
        DM = decision_maker(publisher_msg=None, publishing_topic="/cmd_vel", qos_publisher=odom_qos, goalPoint=[[1.0, 1.0], [2.0, 2.0]])
    else:
        print("Invalid motion type", file=sys.stderr)

    try:
        spin(DM)
    except SystemExit:
        print(f"reached there successfully {DM.localizer.getPose()}")


if __name__=="__main__":

    argParser=argparse.ArgumentParser(description="point or trajectory") 
    argParser.add_argument("--motion", type=str, default="point")
    args = argParser.parse_args()

    main(args)
