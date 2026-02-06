#!/usr/bin/env python3
import time
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

def main():
    # 1. Start the ROS 2 Python Client
    rclpy.init()

    # 2. Create the Navigator Object
    navigator = BasicNavigator()

    # 3. Wait for Nav2 to fully wake up
    # (The robot must be localized in RViz before running this script!)
    print("Waiting for Nav2 to activate...")
    navigator.waitUntilNav2Active()

    # 4. Define our Patrol Points (Relative to the Map Frame)
    # POINT A (x=1.5, y=0.0)
    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = 1.5
    goal_pose1.pose.position.y = 0.0
    goal_pose1.pose.orientation.w = 1.0

    # POINT B (x=0.0, y=0.0) - Back to Start
    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = 0.0
    goal_pose2.pose.position.y = 0.0
    goal_pose2.pose.orientation.w = 1.0

    # 5. The Patrol Loop
    while True:
        # --- GO TO POINT A ---
        print("Going to Point A...")
        navigator.goToPose(goal_pose1)

        while not navigator.isTaskComplete():
            # Print feedback while moving
            feedback = navigator.getFeedback()
            # print(f"Distance remaining: {feedback.distance_remaining:.2f} meters")
            
        # Check result
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print("Reached Point A! Waiting 3 seconds...")
            time.sleep(3)
        else:
            print("Failed to reach Point A!")
            break

        # --- GO TO POINT B ---
        print("Going to Point B...")
        navigator.goToPose(goal_pose2)

        while not navigator.isTaskComplete():
            pass # Just wait
            
        if navigator.getResult() == TaskResult.SUCCEEDED:
            print("Reached Point B! Waiting 3 seconds...")
            time.sleep(3)
        else:
            print("Failed to reach Point B!")
            break

    # Shut down cleanly
    rclpy.shutdown()

if __name__ == '__main__':
    main()