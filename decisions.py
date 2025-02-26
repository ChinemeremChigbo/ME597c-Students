# Imports


import sys

from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error
from pid import PID_ctrl

from rclpy import init, spin, spin_once
from rclpy.node import Node
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Odometry as odom

from localization import localization, rawSensor

from planner import TRAJECTORY_PLANNER, POINT_PLANNER, planner
from controller import controller, trajectoryController

# You may add any other imports you may need/want to use below
# import ...


class decision_maker(Node):
    
    def __init__(self, publisher_msg, publishing_topic, qos_publisher, goalPoint, rate=10, motion_type=POINT_PLANNER):

        super().__init__("decision_maker")

        #TODO Part 4: Create a publisher for the topic responsible for robot's motion
        self.publisher = self.create_publisher(Twist, publishing_topic, qos_publisher)

        publishing_period=1/rate
        
        # Instantiate the controller
        # TODO Part 5: Tune your parameters here
    
        if motion_type == POINT_PLANNER:
            self.controller = controller(klp=0.3, klv=0.6, kap=1.0, kav=0.8)
            self.planner = planner(POINT_PLANNER)
    
        elif motion_type == TRAJECTORY_PLANNER:
            self.controller = trajectoryController(klp=0.3, klv=0.6, kap=1.0, kav=0.8)
            self.planner = planner(TRAJECTORY_PLANNER)
        else:
            print("Error! you don't have this planner", file=sys.stderr)


        # Instantiate the localization, use rawSensor for now  
        self.localizer=localization(rawSensor)

        # Instantiate the planner
        # NOTE: goalPoint is used only for the pointPlanner
        self.goal=self.planner.plan(goalPoint)
        self.current_goal_index = 0 if motion_type == TRAJECTORY_PLANNER else None

        self.create_timer(publishing_period, self.timerCallback)


    def timerCallback(self):
        
        # TODO Part 3: Run the localization node
        ...    # Remember that this file is already running the decision_maker node.
        spin_once(self.localizer)

        if self.localizer.getPose()  is  None:
            print("waiting for odom msgs ....")
            return

        vel_msg=Twist()
        
        # TODO Part 3: Check if you reached the goal
        if self.current_goal_index is not None:
            # We are in trajectory planner mode (goal is a sequence of points)
            current_goal = self.goal[self.current_goal_index]

            # Check if the current goal is reached
            reached_goal = calculate_linear_error(self.localizer.getPose(), current_goal) < 0.05
        else:
            # Point planner mode
            reached_goal = calculate_linear_error(self.localizer.getPose(), self.goal) < 0.05
        print(self.goal)
        if reached_goal:
            print(f"Reached goal: {current_goal}" if self.current_goal_index is not None else "Reached point goal")
            # If in trajectory planner mode, move to the next goal if any
            if self.current_goal_index is not None and self.current_goal_index < len(self.goal) - 1:
                self.current_goal_index += 1
                
                print(f"Moving to next goal: {self.goal[self.current_goal_index]}, {self.current_goal_index + 1}/{len(self.goal)}")
            else:
                self.publisher.publish(vel_msg)  # Stop the robot after reaching the final goal
                self.controller.PID_angular.logger.save_log()
                self.controller.PID_linear.logger.save_log()
                sys.exit(0)

        
        velocity, yaw_rate = self.controller.vel_request(self.localizer.getPose(), self.goal, True)

        #TODO Part 4: Publish the velocity to move the robot
        vel_msg.linear.x = velocity
        vel_msg.angular.z = yaw_rate
        self.publisher.publish(vel_msg)

import argparse


def main(args=None):
    
    init()

    # TODO Part 3: You migh need to change the QoS profile based on whether you're using the real robot or in simulation.
    # Remember to define your QoS profile based on the information available in "ros2 topic info /odom --verbose" as explained in Tutorial 3
    
    odom_qos = QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
    )
    

    # TODO Part 4: instantiate the decision_maker with the proper parameters for moving the robot
    if args.motion.lower() == "point":
        DM = decision_maker(publisher_msg=Twist(), publishing_topic="/cmd_vel", qos_publisher=odom_qos, goalPoint=[1.0, 1.0], motion_type=POINT_PLANNER)
    elif args.motion.lower() == "trajectory":
        DM = decision_maker(publisher_msg=Twist(), publishing_topic="/cmd_vel", qos_publisher=odom_qos, goalPoint=None, motion_type=TRAJECTORY_PLANNER)

    else:
        print("invalid motion type", file=sys.stderr)     
        return   
    try:
        spin(DM)
    except SystemExit:
        print(f"reached there successfully {DM.localizer.pose}")


if __name__=="__main__":

    argParser=argparse.ArgumentParser(description="point or trajectory") 
    argParser.add_argument("--motion", type=str, default="point")
    args = argParser.parse_args()

    main(args)
