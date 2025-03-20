import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import String  # 发送颜色信息
from std_msgs.msg import Float32MultiArray  # 发送目标坐标 (X, Y, Z)

# 定义 7 种颜色的 HSV 颜色范围
color_ranges = {
    "pink":   ([140, 100, 100], [170, 255, 255]),  # 粉色
    "red":    ([0, 120, 70], [10, 255, 255]),      # 红色
    "yellow": ([20, 100, 100], [30, 255, 255]),    # 黄色
    "orange": ([10, 100, 100], [20, 255, 255]),    # 橙色
    "green":  ([35, 100, 100], [85, 255, 255]),    # 绿色
    "blue":   ([90, 50, 70], [130, 255, 255]),     # 蓝色
    "purple": ([130, 50, 70], [160, 255, 255]),    # 紫色
}

class ImageDepthSubscriber(Node):
    def __init__(self, name):
        super().__init__(name)
        
        self.sub_color = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.listener_callback_color, 10
        )
        self.sub_depth = self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw', self.listener_callback_depth, 10
        )

        self.cv_bridge = CvBridge()
        self.depth_image = None
        self.pub_position = self.create_publisher(Float32MultiArray, 'object_center_distance', 10)
        self.pub_color = self.create_publisher(String, 'object_color', 10)

    def listener_callback_color(self, data):
        """ 处理 RGB 图像 """
        self.get_logger().info('Receiving color video frame')
        color_image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')
        self.object_detect(color_image)

    def listener_callback_depth(self, data):
        """ 处理 深度图像 """
        self.get_logger().info('Receiving depth video frame')
        self.depth_image = self.cv_bridge.imgmsg_to_cv2(data, '16UC1')

    def object_detect(self, image):
        """ 目标检测：检测颜色物体，并计算 3D 坐标 """
        if self.depth_image is None:
            return

        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        detected_objects = []

        for color_name, (lower, upper) in color_ranges.items():
            mask = cv2.inRange(hsv_img, np.array(lower), np.array(upper))

            # 形态学操作，减少噪声 & 连接断裂的区域
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                if cv2.contourArea(cnt) < 1000:  # 过滤小目标，防止误检
                    continue

                (x, y, w, h) = cv2.boundingRect(cnt)
                center_x, center_y = int(x + w / 2), int(y + h / 2)

                if center_y >= self.depth_image.shape[0] or center_x >= self.depth_image.shape[1]:
                    continue

                # 获取深度数据
                distance_mm = self.depth_image[center_y, center_x].astype(float)
                distance_meters = distance_mm / 1000.0

                # 过滤不在合理深度范围内的目标
                if distance_meters < 0.1 or distance_meters > 1.5:
                    continue

                detected_objects.append((center_x, center_y, distance_meters, color_name))

                # 画出轮廓和中心点
                cv2.drawContours(image, [cnt], -1, (0, 255, 0), 2)
                cv2.circle(image, (center_x, center_y), 5, (0, 255, 0), -1)
                cv2.putText(image, f"{color_name} {distance_meters:.2f}m", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        for obj in detected_objects:
            center_x, center_y, distance, color = obj
            msg_position = Float32MultiArray()
            msg_position.data = [float(center_x), float(center_y), float(distance)]
            self.pub_position.publish(msg_position)

            msg_color = String()
            msg_color.data = color
            self.pub_color.publish(msg_color)

        cv2.imshow("object", image)
        cv2.waitKey(10)


def main(args=None):
    rclpy.init(args=args)
    node = ImageDepthSubscriber("image_depth_sub")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
