#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
#from tf_transformations import quaternion_matrix
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class IMUTFBroadcaster(Node):
    def __init__(self):
        super().__init__('imu_tf_broadcaster')

        self.br = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10)

    def imu_callback(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'imu_link'

        # IMU orientation
        t.transform.rotation = msg.orientation

        # Keep original translation (0,0,0.03)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.03

        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = IMUTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

