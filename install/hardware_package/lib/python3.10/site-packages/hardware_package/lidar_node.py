import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarSensor(Node):
    """Node that will publish filtered LIDAR data back to the topic."""

    def __init__(self):
        super().__init__("lidar_sensor_node")

        self.lidar_data = None

        self.lidar_data_subscriber = self.create_subscription(
            msg_type=LaserScan,
            topic='/scan',
            callback=self.lidar_subscriber_callback,
            qos_profile=10
        )

        self.lidar_data_publisher = self.create_publisher(
            msg_type=LaserScan,
            topic='/scan_filtered',
            qos_profile=10
        )

        self.timer = self.create_timer(0.5, self.lidar_data_to_publish)

    def lidar_subscriber_callback(self, msg):
        self.lidar_data = msg

    def lidar_data_to_publish(self):
        if self.lidar_data is not None:
            lidar_message = LaserScan()
            lidar_message = self.lidar_data

            self.lidar_data_publisher.publish(lidar_message)


def main(args=None):
    try:
        rclpy.init(args=args)

        lidar_sensor_node = LidarSensor()

        rclpy.spin(lidar_sensor_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()
