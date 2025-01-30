import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class RealsenseCamera(Node):
    """
    Node that republishes RealSense Camera's depth color points to a specific topic.
    """

    def __init__(self):
        super().__init__("realsense_point_cloud_node")

        self.point_cloud_data = None

        # Subscriber to the RealSense point cloud topic
        self.point_cloud_subscriber = self.create_subscription(
            PointCloud2,
            '/camera/depth/color/points',
            self.point_cloud_callback,
            10
        )

        # Publisher for republishing the point cloud data
        self.point_cloud_publisher = self.create_publisher(
            PointCloud2,
            '/camera/depth/color/points',
            10
        )

        # Timer to periodically republish the data
        self.timer = self.create_timer(0.5, self.publish_point_cloud)

    def point_cloud_callback(self, msg):
        """Callback function to store incoming point cloud data."""
        self.get_logger().info("PointCloud2 data received.")
        self.point_cloud_data = msg

    def publish_point_cloud(self):
        """Publishes the stored point cloud data to the target topic."""
        if self.point_cloud_data is not None:
            self.point_cloud_publisher.publish(self.point_cloud_data)
            self.get_logger().info("Published PointCloud2 data.")


def main(args=None):
    rclpy.init(args=args)

    realsense_point_cloud_node = RealsenseCamera()

    try:
        rclpy.spin(realsense_point_cloud_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        realsense_point_cloud_node.get_logger().error(f"Exception occurred: {e}")
    finally:
        realsense_point_cloud_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
