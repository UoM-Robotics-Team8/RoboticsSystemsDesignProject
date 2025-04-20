import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import String, Float32MultiArray
import time

# 颜色范围保持不变
color_ranges = {
    "red":    ([0, 185, 50], [15, 255, 255]),
    "yellow": ([22, 155, 50], [35, 255, 255]),
    "blue":   ([90, 190, 80], [110, 255, 255])
}

class ImageDepthSubscriber(Node):
    def __init__(self, name):
        super().__init__(name)
        
        # 订阅相机内参
        self.sub_camera_info = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10
        )
        
        # 图像订阅
        self.sub_color = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.listener_callback_color, 10
        )
        self.sub_depth = self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw', self.listener_callback_depth, 10
        )

        self.cv_bridge = CvBridge()
        self.depth_image = None
        self.latest_target = None
        self.last_publish_time = 0
        
        # 相机内参初始化
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.camera_params_ready = False

        # 创建发布器
        self.pub_position = self.create_publisher(Float32MultiArray, 'object_center_distance', 10)
        self.pub_color = self.create_publisher(String, 'object_color', 10)

        # 创建10Hz定时器
        self.create_timer(0.1, self.publish_target)

    def camera_info_callback(self, msg):
        """ 处理相机内参 """
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.camera_params_ready = True
        self.get_logger().info(f'Received camera params: fx={self.fx}, fy={self.fy}')

    def pixel_to_3d(self, x, y, z):
        """ 像素坐标转真实3D坐标 (米) """
        if not self.camera_params_ready or z == 0:
            return (0.0, 0.0, 0.0)
        x_3d = (x - self.cx) * z / self.fx
        y_3d = (y - self.cy) * z / self.fy
        return (x_3d, y_3d, z)

    def listener_callback_color(self, data):
        """ 处理RGB图像 """
        try:
            color_image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')
            self.object_detect(color_image)
        except Exception as e:
            self.get_logger().error(f'Color processing error: {str(e)}')

    def listener_callback_depth(self, data):
        """ 处理深度图像 """
        try:
            self.depth_image = self.cv_bridge.imgmsg_to_cv2(data, '16UC1')
        except Exception as e:
            self.get_logger().error(f'Depth processing error: {str(e)}')

    def object_detect(self, image):
        if self.depth_image is None or not self.camera_params_ready:
            return

        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        detected_objects = []

        for color_name, (lower, upper) in color_ranges.items():
            mask = cv2.inRange(hsv_img, np.array(lower), np.array(upper))
            
            # 形态学操作
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                if cv2.contourArea(cnt) < 1000:
                    continue

                # 获取边界框和中心点
                x, y, w, h = cv2.boundingRect(cnt)
                center_x = x + w//2
                center_y = y + h//2

                # 边界检查
                if center_y >= self.depth_image.shape[0] or center_x >= self.depth_image.shape[1]:
                    continue

                # 获取深度值
                distance_mm = self.depth_image[center_y, center_x]
                distance_m = distance_mm / 1000.0

                # 过滤无效值
                if distance_m < 0.1 or distance_m > 1.5 or distance_mm == 0:
                    continue

                # 坐标转换
                x_3d, y_3d, z_3d = self.pixel_to_3d(center_x, center_y, distance_m)

                detected_objects.append({
                    'pixel': (center_x, center_y),
                    'position': (x_3d, y_3d, z_3d),
                    'color': color_name
                })

                # 绘制可视化元素（保持原有绘图代码）
                cv2.drawContours(image, [cnt], -1, (0, 255, 0), 2)
                cv2.circle(image, (center_x, center_y), 5, (0, 255, 0), -1)
                cv2.putText(image, f"{color_name} {z_3d:.2f}m", 
                          (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # 显示结果图像
        cv2.imshow("Detection Preview", image)
        cv2.waitKey(1)

        # 选择最近目标
        if detected_objects:
            closest = min(detected_objects, key=lambda x: x['position'][2])
            self.latest_target = {
                'position': closest['position'],
                'color': closest['color'],
                'pixel': closest['pixel']
            }

    def publish_target(self):
        """ 定时发布最新目标 """
        if self.latest_target and (time.time() - self.last_publish_time) > 0.1:
            # 发布3D坐标
            msg_pos = Float32MultiArray()
            msg_pos.data = list(self.latest_target['position'])
            self.pub_position.publish(msg_pos)
            
            # 发布颜色信息
            msg_color = String()
            msg_color.data = self.latest_target['color']
            self.pub_color.publish(msg_color)
            
            self.last_publish_time = time.time()

def main(args=None):
    rclpy.init(args=args)
    node = ImageDepthSubscriber("object_detector")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()