import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray

class ObjectDetectListener(Node):
   """Node will listen to the distance to detected object, pause m-explore
      then use Nav2 to navigate towards the detcted object""" 
   
    def __init__(self):
      super().__init__("object_detect_listener_node")

      self.object_detected: bool = False

      self.object_distance_subscriber = self.create_subscription(
         msg_type=Float32MultiArray,
         topic='/object_center_distance',
         callback=self.distance_to_object
         qos_profile=10
      )

      self.create_timer(0.1, )

    def distance_to_object(self, msg):
     if 
    
    def nav_to_object(self, )