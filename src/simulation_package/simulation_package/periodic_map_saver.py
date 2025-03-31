import rclpy
from rclpy.node import Node
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix

############################################################
# NOTE: THIS SAVES TO THE INSTALL FOLDER, NOT THE SRC FOLDER
############################################################

MAP_PATH = os.path.join(get_package_share_directory('simulation_package'), 'maps')

class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver')
        self.save_interval = 10  # Save map every 10 seconds
        self.map_path = os.path.join(MAP_PATH, 'auto_saved_map.pgm')  # Relative path
        self.timer = self.create_timer(self.save_interval, self.save_map)

    def save_map(self):
        os.system(f"ros2 run nav2_map_server map_saver_cli -f {self.map_path}")
        self.get_logger().info(f"Saved map: {self.map_path}")

def main(args=None):
    try:
        rclpy.init(args=args)

        map_saver_node = MapSaver()

        rclpy.spin(map_saver_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)

if __name__ == '__main__':
    main()
