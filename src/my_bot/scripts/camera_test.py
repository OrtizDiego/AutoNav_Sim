#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        
        # Create the Subscriber
        self.subscription = self.create_subscription(
            Image, 
            '/camera/image_raw', 
            self.listener_callback, 
            10)
        
        # The Bridge object converts ROS messages to OpenCV images
        self.bridge = CvBridge()
        
    def listener_callback(self, msg):
        try:
            # 1. Convert ROS Image to OpenCV Image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 2. Process the Image (Draw a Green Circle in the center)
            height, width, channels = cv_image.shape
            center_x = int(width / 2)
            center_y = int(height / 2)
            
            # Draw: (Image, Center, Radius, Color(BGR), Thickness)
            cv2.circle(cv_image, (center_x, center_y), 50, (0, 255, 0), 3)
            cv2.putText(cv_image, "I CAN SEE!", (center_x - 60, center_y - 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # 3. Display the Image
            cv2.imshow("Robot Camera View", cv_image)
            cv2.waitKey(1) # Refresh the window
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()