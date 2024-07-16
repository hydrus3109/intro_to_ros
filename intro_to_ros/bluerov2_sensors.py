#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import BatteryState, Imu

import numpy as np
class ROVSubscriber(Node):
    def __init__(self):
        super().__init__("rov_subscriber")
                
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        self.battery_subscription = self.create_subscription(
            BatteryState,
            '/mavros/battery',
            self.battery_callback,
            qos_profile
        )
        self.imu_subscription = self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.imu_callback,
            qos_profile
        )
        self.battery_subscription
        self.imu_subscription
        self.battery_state = BatteryState()
        self.imu_state = Imu()
        self.timer = self.create_timer(5.0, self.battery_check)

    def battery_callback(self, msg):
        self.battery_state = msg
        self.get_logger().info(str(msg.voltage))
    def imu_callback(self, msg):
        self.imu_state = msg

    def battery_check(self):
        if self.battery_state.voltage < 3.0:  
            self.get_logger().warn('Battery voltage is below safe levels!')
        
def main(args=None):
    rclpy.init(args=args)
    node = ROVSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__=="__main__":
    main()