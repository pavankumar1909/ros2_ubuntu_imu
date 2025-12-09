"""
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
"""

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

        # Initialize I2C
        self.bus = smbus.SMBus(1)
        self.bus.write_byte_data(MPU_ADDR, PWR_MGMT, 0)
        
        # Publisher
        self.pub = self.create_publisher(Imu, "/imu/data", 50)
        self.timer = self.create_timer(0.05, self.loop)  # 20 Hz
        
        self.prev_ax, self.prev_ay, self.prev_az = 0.0, 0.0, 0.0
        self.prev_gx, self.prev_gy, self.prev_gz = 0.0, 0.0, 0.0
        
        # Mahony filter variables
        self.q0 = 1.0
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0
        
        self.kp = 0.5    #proportional gain
        self.ki = 0.001  #integral gain
        self.eInt = [0.0, 0.0, 0.0]
        
        self.get_logger().info("IMU Mahony node started")

    def mahony_update(self, ax, ay, az, gx, gy, gz, dt):
        # Normalize accelerometer
        norm = math.sqrt(ax*ax + ay*ay + az*az)
        if norm == 0:
            return
        ax /= norm
        ay /= norm
        az /= norm

        # Estimated direction of gravity
        vx = 2*(self.q1*self.q3 - self.q0*self.q2)
        vy = 2*(self.q0*self.q1 + self.q2*self.q3)
        vz = self.q0*self.q0 - self.q1*self.q1 - self.q2*self.q2 + self.q3*self.q3

        # Error = cross product of estimated vs measured gravity
        ex = (ay*vz - az*vy)
        ey = (az*vx - ax*vz)
        ez = (ax*vy - ay*vx)

        # Integral error
        self.eInt[0] += ex * dt
        self.eInt[1] += ey * dt
        self.eInt[2] += ez * dt

        # Apply feedback
        gx += self.kp*ex + self.ki*self.eInt[0]
        gy += self.kp*ey + self.ki*self.eInt[1]
        gz += self.kp*ez + self.ki*self.eInt[2]

        # Integrate rate of change of quaternion
        gx *= 0.5*dt
        gy *= 0.5*dt
        gz *= 0.5*dt

        qa = self.q0
        qb = self.q1
        qc = self.q2
        qd = self.q3

        self.q0 += -qb*gx - qc*gy - qd*gz
        self.q1 +=  qa*gx + qc*gz - qd*gy
        self.q2 +=  qa*gy - qb*gz + qd*gx
        self.q3 +=  qa*gz + qb*gy - qc*gx

        # Normalize quaternion
        norm_q = math.sqrt(self.q0*self.q0 + self.q1*self.q1 + self.q2*self.q2 + self.q3*self.q3)
        self.q0 /= norm_q
        self.q1 /= norm_q
        self.q2 /= norm_q
        self.q3 /= norm_q

    def loop(self):
        imu_msg = Imu()
        alpha = 0.5 # smoothing factor
        # Read raw values 
       
        ax = read_word(self.bus, MPU_ADDR, ACCEL_XOUT) / 16384.0
        ay = read_word(self.bus, MPU_ADDR, ACCEL_YOUT) / 16384.0
        az = read_word(self.bus, MPU_ADDR, ACCEL_ZOUT) / 16384.0

        gx = read_word(self.bus, MPU_ADDR, GYRO_XOUT) / 131.0
        gy = read_word(self.bus, MPU_ADDR, GYRO_YOUT) / 131.0
        gz = read_word(self.bus, MPU_ADDR, GYRO_ZOUT) / 131.0
        
        # Accelerometer
        ax = alpha*ax + ( 1 - alpha) * self.prev_ax
        ay = alpha*ay + ( 1 - alpha) * self.prev_ay
        az = alpha*az + ( 1- alpha ) * self.prev_az 

        #Gyroscope
        gx = alpha*gx + ( 1 - alpha) * self.prev_gx
        gy = alpha*gy + ( 1 - alpha) * self.prev_gy
        gz = alpha*gz + ( 1-  alpha) * self.prev_gz

        # Save for next iteration
        self.prev_ax, self.prev_ay, self.prev_az = ax, ay, az
        self.prev_gx, self.prev_gy, self.prev_gz = gx, gy, gz         

        # Mahony update
        dt = 0.05  # 20 Hz
        self.mahony_update(ax, ay, az, gx, gy, gz, dt)

        # Populate IMU message
        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az

        imu_msg.angular_velocity.x = gx
        imu_msg.angular_velocity.y = gy
        imu_msg.angular_velocity.z = gz

        imu_msg.orientation.w = self.q0
        imu_msg.orientation.x = self.q1
        imu_msg.orientation.y = self.q2
        imu_msg.orientation.z = self.q3

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

