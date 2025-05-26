import rclpy
from rclpy.node import Node
from pointcloud_to_laserscan import PointCloudToLaserScan

class PointCloudToLaserScanNode(Node):
    def __init__(self):
        super().__init__('pointcloud_to_laserscan_node')

        # Parameters for the PointCloud2 to LaserScan conversion
        self.declare_parameter('cloud_topic', '/camera/camera/depth/color/points')  # Replace with your PointCloud2 topic
        self.declare_parameter('frame_id', 'camera_camera_link')
        self.declare_parameter('angle_min', -1.5708)  # -90 degrees
        self.declare_parameter('angle_max', 1.5708)  # 90 degrees
        self.declare_parameter('range_max', 1.0)  # 1 meter max range

        # Create the pointcloud_to_laserscan node
        self.laserscan_node = PointCloudToLaserScan(self)

        # Create and configure the parameters for pointcloud_to_laserscan
        self.laserscan_node.set_parameters([
            rclpy.parameter.Parameter('cloud_topic', rclpy.Parameter.Type.STRING, self.get_parameter('cloud_topic').value),
            rclpy.parameter.Parameter('frame_id', rclpy.Parameter.Type.STRING, self.get_parameter('frame_id').value),
            rclpy.parameter.Parameter('angle_min', rclpy.Parameter.Type.DOUBLE, self.get_parameter('angle_min').value),
            rclpy.parameter.Parameter('angle_max', rclpy.Parameter.Type.DOUBLE, self.get_parameter('angle_max').value),
            rclpy.parameter.Parameter('range_max', rclpy.Parameter.Type.DOUBLE, self.get_parameter('range_max').value),
        ])

        self.get_logger().info("PointCloud to LaserScan node is running.")

def main(args=None):
    rclpy.init(args=args)

    # Create the node and spin it

    try:
        rclpy.init(args=args)

        pointcloud_to_laserscan_node = PointCloudToLaserScanNode()

        rclpy.spin(pointcloud_to_laserscan_node)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)

if __name__ == '__main__':
    main()
