import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import String, Float32MultiArray, Bool
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PointStamped
from math import atan2, sin, cos
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from rclpy.time import Time


class ObjectDetectListener(Node):
   """Node will listen to the distance to detected object, pause m-explore
      then use Nav2 to navigate towards the detcted object""" 
   
   def __init__(self):
      super().__init__("object_detect_listener_node")

      self.objects_detected: int = 0
      self.no_of_objects: int = 0
      self.dist_msg: Float32MultiArray = None
      self.object_x: float = None
      self.object_y: float = None

      self.tf_buffer = Buffer()
      self.tf_listener = TransformListener(self.tf_buffer, self)

      self.object_distance_subscriber = self.create_subscription(
         msg_type=Float32MultiArray,
         topic='/object_center_distance',
         callback=self.object_callback,
         qos_profile=10
      )

      # self.object_grabbed_subscriber = self.create_subscription(
      #    msg_type=Bool,
      #    topic='/object_grabbed',
      #    callback=self.object_grabbed_callback,
      #    qos_profile=1
      # )

      self.explore_publisher = self.create_publisher(
         msg_type=Bool,
         topic='/explore/resume',
         qos_profile=1
      )

      self.nav2_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

      self.future: Future = None

   def object_callback(self, msg):
      self.dist_msg = msg

      self.distance_to_object()

   # def object_grabbed_callback(self, msg):
   #    if msg.data == True:
   #       self.no_of_objects += 1
   #       if self.no_of_objects == 3:
   #          self.go_home()
   #       else:
   #          self.resume_explore()


   def distance_to_object(self):
      if self.dist_msg.data[2] == 0.0 and self.dist_msg.data[0] == 0.0:
         self.get_logger().info("Waiting for data on /object_center_distance...")
         return
      else:
         if self.objects_detected > 1:

            # pause explore, set goal pose, nav to goal pose
            self.pause_explore()
            
            self.object_x = self.dist_msg.data[2]
            self.object_y = self.dist_msg.data[0]

            self.nav_to_object()
         else:
            self.objects_detected += 1

   def get_map_point(self, object_x, object_y):
      point_in_camera_frame = PointStamped()
      point_in_camera_frame.header.stamp = self.get_clock().now().to_msg()
      point_in_camera_frame.header.frame_id = "camera_camera_link"
      point_in_camera_frame.point.x = object_x
      point_in_camera_frame.point.y = object_y
      point_in_camera_frame.point.z = 0

      try:
         transform = self.tf_buffer.lookup_transform(
            target_frame="map",
            source_frame="camera_camera_link",
            time=Time(),
            timeout=rclpy.duration.Duration(seconds=1.0)
         )

         point_in_map = do_transform_point(point_in_camera_frame, transform)

         return point_in_map.point.x, point_in_map.point.y
      except Exception as e:
         self.get_logger().info("Tranform failed", e)
         return None, None


   def pause_explore(self):
      msg = Bool()
      msg.data = False
      self.explore_publisher.publish(msg)

   def resume_explore(self):
      msg = Bool()
      msg.data = True
      self.explore_publisher.publish(msg)
   
   def nav_to_object(self):

      x_map, y_map = self.get_map_point(self.object_x, self.object_y)

      goal = NavigateToPose.Goal()
      
      # expects this message type
      goal.pose = PoseStamped()

      # set the goal pose frame
      goal.pose.header.frame_id = "map"

      # time is needed in the header
      goal.pose.header.stamp = self.get_clock().now().to_msg()

      # set the goal position (quaternion)
      goal.pose.pose.position.x = x_map
      goal.pose.pose.position.y = y_map
      goal.pose.pose.position.z = 0.0 # 2D

      # set angle so that it is facing the object on arrival
      angle = atan2(y_map, x_map)
      goal.pose.pose.orientation.z = sin(angle/2)
      goal.pose.pose.orientation.w = cos(angle/2)

      self.send_goal(goal)

   def send_goal(self, goal):
      # wait for nav2 to become available
      if not self.nav2_client.wait_for_server(timeout_sec=10.0):
         self.get_logger().info("Nav2 action server not available after 10 seconds")

      # # send goal to nav2  
      # self.future = self.nav2_client.send_goal_async(goal)

      # # listens for the outcome of the navigation goal
      # self.future.add_done_callback(self.goal_response_callback)

      self.nav2_client.send_goal_async(goal).add_done_callback(self.goal_response_callback)

   def goal_response_callback(self, future):
      goal_handle = future.result()
      if not goal_handle.accepted:
         self.get_logger().info("Goal rejected by Nav2.")
         self.resume_explore()
         return
      
      self.get_logger().info("Goal accepted by Nav2, waiting for result...")
      goal_handle.get_result_async().add_done_callback(self.navigation_result_callback)

   def navigation_result_callback(self, future: Future):
      result = future.result()

      if result is None:
         self.get_logger().info("NAVIGATION RESULT NONE!!!")
         self.resume_explore()
         return

      if result.status == 3:
         # navigation successful, so pick up object
         self.get_logger().info("Navigation Succeeded!!")
         self.objects_detected = 0
         return
      elif result.status > 3:
         # navigation unsuccesful - bad!
         self.get_logger().info("ERROR! NAVIGATION FAILED")
         self.get_logger().info(str(result.status))
         # resume explore and hope it'll not fail next time
         self.resume_explore()
         return
      else:
         self.get_logger().info(str(result.status))
         return

def main(args=None):
      try:
         rclpy.init(args=args)

         object_detection_listener_node = ObjectDetectListener()

         rclpy.spin(object_detection_listener_node)

      except KeyboardInterrupt:
         pass
      except Exception as e:
         print(e)


if __name__ == '__main__':
   main()