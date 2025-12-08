#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus
import math
import time

# MPU6050 Registers
MPU_ADDR = 0x68
PWR_MGMT = 0x6B

ACCEL_XOUT = 0x3B
ACCEL_YOUT = 0x3D
ACCEL_ZOUT = 0x3F
GYRO_XOUT  = 0x43
GYRO_YOUT  = 0x45
GYRO_ZOUT  = 0x47

def read_word(bus, addr, reg):
    high = bus.read_byte_data(addr, reg)
    low = bus.read_byte_data(addr, reg + 1)
    value = (high << 8) + low
    return value if value < 32768 else value - 65536

class ImuNode(Node):
    def __init__(self):
        super().__init__("imu_node")

        self.bus = smbus.SMBus(1)
        self.bus.write_byte_data(MPU_ADDR, PWR_MGMT, 0)

        self.pub = self.create_publisher(Imu, "/imu/data", 50)
        self.timer = self.create_timer(0.05, self.loop)  # 20 Hz

        self.get_logger().info("IMU Mahony node started")

    def loop(self):
        imu_msg = Imu()

        # Read raw values
        ax = read_word(self.bus, MPU_ADDR, ACCEL_XOUT) / 16384.0
        ay = read_word(self.bus, MPU_ADDR, ACCEL_YOUT) / 16384.0
        az = read_word(self.bus, MPU_ADDR, ACCEL_ZOUT) / 16384.0

        gx = read_word(self.bus, MPU_ADDR, GYRO_XOUT) / 131.0
        gy = read_word(self.bus, MPU_ADDR, GYRO_YOUT) / 131.0
        gz = read_word(self.bus, MPU_ADDR, GYRO_ZOUT) / 131.0

        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az

        imu_msg.angular_velocity.x = gx
        imu_msg.angular_velocity.y = gy
        imu_msg.angular_velocity.z = gz

        # (Orientation from Mahony filter — optional)
        # For now set default quaternion:
        imu_msg.orientation.w = 1.0
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.0

        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        self.pub.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

