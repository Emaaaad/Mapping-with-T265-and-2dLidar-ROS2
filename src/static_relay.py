#!/usr/bin/env python3
import rclpy, sys
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

class StaticRelay(Node):
    def __init__(self):
        qos_in  = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
        qos_out = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)

        super().__init__('static_relay')
        self.pub = self.create_publisher(TFMessage, '/tf_static', qos_out)
        self.sub = self.create_subscription(
            TFMessage, '/tf_static', self.cb, qos_in)

    def cb(self, msg):
        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(StaticRelay())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
