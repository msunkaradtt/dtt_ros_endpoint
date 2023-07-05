import rclpy
from rclpy.node import Node

import random

from ros_unity_interfaces.msg import Colordtt

class ColorChangePublisher(Node):
    def __init__(self):
        super().__init__("colorChange_Node")
        self.publisher_ = self.create_publisher(Colordtt, "dtt_ros_colorChange_topic", 10)
        
        time_period = 2
        self.get_logger().info(f"Started Publishing The Color Data for Every {time_period} sec.")
        self.timer = self.create_timer(time_period, self.timer_callback)
        
    def timer_callback(self):
        msg = Colordtt()
        msg.r = random.randint(0, 255)
        msg.g = random.randint(0, 255)
        msg.b = random.randint(0, 255)
        msg.a = round(random.random(), 2)

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    colorChange_publisher = ColorChangePublisher()
    rclpy.spin(colorChange_publisher)

    colorChange_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()