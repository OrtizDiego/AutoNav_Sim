#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class BallChaser(Node):
    def __init__(self):
        super().__init__('ball_chaser')
        
        # Subscriber (The Eyes)
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10)
        
        # Publisher (The Feet)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.bridge = CvBridge()

    def camera_callback(self, msg):
        try:
            # 1. Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 2. Convert BGR to HSV (Better for color detection)
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # 3. Define the Range of "Red" Color
            # Red wraps around 180 in HSV, so we need two ranges
            lower_red1 = np.array([0, 100, 100])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([160, 100, 100])
            upper_red2 = np.array([180, 255, 255])
            
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = mask1 + mask2 # Combine

            # 4. Find Contours (Blobs of red)
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            # Command to send to the robot
            cmd = Twist()
            
            if len(contours) > 0:
                # Find the biggest red blob
                c = max(contours, key=cv2.contourArea)
                M = cv2.moments(c)
                
                if M['m00'] > 0:
                    # Find the Center (cx, cy) of the blob
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    
                    # Draw a circle on the image for debugging
                    cv2.circle(cv_image, (cx, cy), 10, (0, 255, 0), 3)

                    # --- CONTROL LOGIC ---
                    height, width, _ = cv_image.shape
                    error_x = cx - (width / 2) # How far from center?
                    
                    # Turn towards the object (Proportional Controller)
                    # Divide by 100 to scale it down
                    cmd.angular.z = -error_x / 100.0 
                    
                    # If we see it, drive forward slowly
                    cmd.linear.x = 0.2
                    
                    print(f"Tracking! Error: {error_x}")
            else:
                # If no red detected, stop (or spin to search)
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            
            # 5. Move the Robot
            self.publisher.publish(cmd)

            # 6. Show the view
            cv2.imshow("Ball Chaser View", cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BallChaser()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()