import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')

        # ROS2 Publishers
        self.image_pub = self.create_publisher(Image, 'color_detected_image', 10)
        self.position_pub = self.create_publisher(Point, 'color_object_position', 10)

        # OpenCV Bridge
        self.bridge = CvBridge()

        # RealSense Pipeline Setup
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # Enable depth stream
        self.pipeline.start(config)

        # Create HSV tuner UI
        self.create_hsv_tuner()

        # Timer for running at 30 FPS
        self.timer = self.create_timer(1/30, self.process_frame)

        self.get_logger().info("Color Detector Node Started")

    def create_hsv_tuner(self):
        """Creates OpenCV Trackbars for HSV tuning."""
        cv2.namedWindow("HSV Tuner")
        cv2.resizeWindow("HSV Tuner", 600, 300)

        # Create trackbars for HSV range adjustment
        cv2.createTrackbar("H Min", "HSV Tuner", 0, 179, lambda x: None)
        cv2.createTrackbar("H Max", "HSV Tuner", 179, 179, lambda x: None)
        cv2.createTrackbar("S Min", "HSV Tuner", 0, 255, lambda x: None)
        cv2.createTrackbar("S Max", "HSV Tuner", 255, 255, lambda x: None)
        cv2.createTrackbar("V Min", "HSV Tuner", 0, 255, lambda x: None)
        cv2.createTrackbar("V Max", "HSV Tuner", 255, 255, lambda x: None)

    def get_hsv_values(self):
        """Reads trackbar values and returns HSV lower & upper range as NumPy arrays."""
        h_min = cv2.getTrackbarPos("H Min", "HSV Tuner")
        h_max = cv2.getTrackbarPos("H Max", "HSV Tuner")
        s_min = cv2.getTrackbarPos("S Min", "HSV Tuner")
        s_max = cv2.getTrackbarPos("S Max", "HSV Tuner")
        v_min = cv2.getTrackbarPos("V Min", "HSV Tuner")
        v_max = cv2.getTrackbarPos("V Max", "HSV Tuner")

        lower_hsv = np.array([h_min, s_min, v_min])
        upper_hsv = np.array([h_max, s_max, v_max])

        return lower_hsv, upper_hsv

    def process_frame(self):
        # Get frames from RealSense
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            return  # Skip if no frame available

        # Convert frame to numpy array
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # Convert to HSV
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        # Get dynamic HSV values
        lower_hsv, upper_hsv = self.get_hsv_values()

        # Create mask and apply it
        mask = cv2.inRange(hsv_image, lower_hsv, upper_hsv)
        result = cv2.bitwise_and(color_image, color_image, mask=mask)

        # Show the filtered image
        cv2.imshow("Filtered Image", result)
        cv2.imshow("Original Image", color_image)

        # Exit if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

        # Publish the processed image
        image_msg = self.bridge.cv2_to_imgmsg(result, encoding='bgr8')
        self.image_pub.publish(image_msg)

    def destroy_node(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

