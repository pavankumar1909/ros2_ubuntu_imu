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
'''
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


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus
import math
import time

# ----- MPU6050 Registers -----
MPU_ADDR = 0x68
PWR_MGMT = 0x6B

ACCEL_XOUT = 0x3B
ACCEL_YOUT = 0x3D
ACCEL_ZOUT = 0x3F
GYRO_XOUT  = 0x43
GYRO_YOUT  = 0x45
GYRO_ZOUT  = 0x47

# Constants
G_TO_M_S2 = 9.80665  # 1 g = 9.80665 m/s^2
DEG2RAD = math.pi/180.0

def read_word(bus, addr, reg):
    high = bus.read_byte_data(addr, reg)
    low = bus.read_byte_data(addr, reg + 1)
    value = (high << 8) + low
    return value if value < 32768 else value - 65536

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        # I2C init
        self.bus = smbus.SMBus(1)
        self.bus.write_byte_data(MPU_ADDR, PWR_MGMT, 0)  # wake up

        # Publisher & timer
        self.pub = self.create_publisher(Imu, '/imu/data', 50)

        # Tuning parameters (experiment)
        self.rate_hz = 50.0          # recommended: 50 - 200 Hz
        self.dt = 1.0 / self.rate_hz
        self.timer = self.create_timer(self.dt, self.loop)

        # Mahony filter variables (quaternion)
        self.q0 = 1.0
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0

        # Mahony gains (start here, tune)
        self.kp = 0.5   # proportional gain
        self.ki = 0.0   # integral gain (zero initially)
        self.eInt = [0.0, 0.0, 0.0]

        # Low-pass filter alpha (exponential smoothing): 0 < alpha <= 1
        # alpha close to 1 -> more responsive (less smoothing)
        # try values 0.4 - 0.8
        self.alpha = 0.6

        # LPF previous values (init)
        self.prev_ax = 0.0
        self.prev_ay = 0.0
        self.prev_az = 0.0
        self.prev_gx = 0.0
        self.prev_gy = 0.0
        self.prev_gz = 0.0

        # Gyro bias (calibration at startup)
        self.gyro_bias = [0.0, 0.0, 0.0]
        self.calibrated = False
        self.calibration_samples = int(1.5 * self.rate_hz)  # 1.5 seconds of samples
        self.cal_count = 0
        self.cal_sum = [0.0, 0.0, 0.0]

        # timing
        self.prev_time = time.monotonic()

        self.get_logger().info('IMU Mahony node started — calibrating gyro for {:.1f}s'.format(self.calibration_samples / self.rate_hz))

    def calibrate_gyro_step(self, gx, gy, gz):
        # accumulate for initial bias estimation
        self.cal_sum[0] += gx
        self.cal_sum[1] += gy
        self.cal_sum[2] += gz
        self.cal_count += 1
        if self.cal_count >= self.calibration_samples:
            self.gyro_bias[0] = self.cal_sum[0] / self.cal_count
            self.gyro_bias[1] = self.cal_sum[1] / self.cal_count
            self.gyro_bias[2] = self.cal_sum[2] / self.cal_count
            self.calibrated = True
            self.get_logger().info('Gyro calibrated: bias = [{:.6f}, {:.6f}, {:.6f}] (rad/s)'.format(*self.gyro_bias))

    def mahony_update(self, ax, ay, az, gx, gy, gz, dt):
        # ax,ay,az are accel in g-units (or unitless) but normalized inside
        # gx,gy,gz in rad/s
        # 1) normalize accel
        norm = math.sqrt(ax*ax + ay*ay + az*az)
        if norm == 0:
            return
        axn = ax / norm
        ayn = ay / norm
        azn = az / norm

        # 2) estimated gravity direction from quaternion
        vx = 2.0*(self.q1*self.q3 - self.q0*self.q2)
        vy = 2.0*(self.q0*self.q1 + self.q2*self.q3)
        vz = self.q0*self.q0 - self.q1*self.q1 - self.q2*self.q2 + self.q3*self.q3

        # 3) error = measured x estimated (cross product)
        ex = (ayn * vz - azn * vy)
        ey = (azn * vx - axn * vz)
        ez = (axn * vy - ayn * vx)

        # 4) integral
        self.eInt[0] += ex * dt
        self.eInt[1] += ey * dt
        self.eInt[2] += ez * dt

        # 5) apply feedback to gyro
        gx_corr = gx + self.kp * ex + self.ki * self.eInt[0]
        gy_corr = gy + self.kp * ey + self.ki * self.eInt[1]
        gz_corr = gz + self.kp * ez + self.ki * self.eInt[2]

        # 6) integrate quaternion (short Euler step)
        gx2 = gx_corr * (0.5 * dt)
        gy2 = gy_corr * (0.5 * dt)
        gz2 = gz_corr * (0.5 * dt)

        qa = self.q0
        qb = self.q1
        qc = self.q2
        qd = self.q3

        self.q0 += -qb*gx2 - qc*gy2 - qd*gz2
        self.q1 +=  qa*gx2 + qc*gz2 - qd*gy2
        self.q2 +=  qa*gy2 - qb*gz2 + qd*gx2
        self.q3 +=  qa*gz2 + qb*gy2 - qc*gx2

        # 7) normalize quaternion
        norm_q = math.sqrt(self.q0*self.q0 + self.q1*self.q1 + self.q2*self.q2 + self.q3*self.q3)
        if norm_q == 0:
            return
        self.q0 /= norm_q
        self.q1 /= norm_q
        self.q2 /= norm_q
        self.q3 /= norm_q

    def loop(self):
        # read timestamp & compute dt (monotonic)
        now = time.monotonic()
        dt = now - self.prev_time
        if dt <= 0:
            dt = self.dt
        self.prev_time = now

        # read raw sensor values
        raw_ax = read_word(self.bus, MPU_ADDR, ACCEL_XOUT)
        raw_ay = read_word(self.bus, MPU_ADDR, ACCEL_YOUT)
        raw_az = read_word(self.bus, MPU_ADDR, ACCEL_ZOUT)

        raw_gx = read_word(self.bus, MPU_ADDR, GYRO_XOUT)
        raw_gy = read_word(self.bus, MPU_ADDR, GYRO_YOUT)
        raw_gz = read_word(self.bus, MPU_ADDR, GYRO_ZOUT)

        # convert to physical units:
        # accelerometer: raw/16384 -> g (for +/-2g)
        ax_g = raw_ax / 16384.0
        ay_g = raw_ay / 16384.0
        az_g = raw_az / 16384.0

        # provide acceleration in m/s^2 for message
        ax_mps2 = ax_g * G_TO_M_S2
        ay_mps2 = ay_g * G_TO_M_S2
        az_mps2 = az_g * G_TO_M_S2

        # gyro: raw/131 -> deg/s (for +/-250deg/s); convert to rad/s:
        gx_deg = raw_gx / 131.0
        gy_deg = raw_gy / 131.0
        gz_deg = raw_gz / 131.0

        gx = gx_deg * DEG2RAD
        gy = gy_deg * DEG2RAD
        gz = gz_deg * DEG2RAD

        # Calibration stage: while not calibrated, accumulate gyro samples
        if not self.calibrated:
            # apply a small LPF to raw gyro before accumulation if needed
            self.calibrate_gyro_step(gx, gy, gz)
            # publish zero orientation during calibration optionally
            if not self.calibrated:
                return  # wait until calibrated

        # subtract gyro bias (computed during calibration)
        gx -= self.gyro_bias[0]
        gy -= self.gyro_bias[1]
        gz -= self.gyro_bias[2]

        # low-pass filter (exponential)
        alpha = self.alpha
        ax_g = alpha * ax_g + (1.0 - alpha) * self.prev_ax
        ay_g = alpha * ay_g + (1.0 - alpha) * self.prev_ay
        az_g = alpha * az_g + (1.0 - alpha) * self.prev_az

        gx = alpha * gx + (1.0 - alpha) * self.prev_gx
        gy = alpha * gy + (1.0 - alpha) * self.prev_gy
        gz = alpha * gz + (1.0 - alpha) * self.prev_gz

        # save prev
        self.prev_ax, self.prev_ay, self.prev_az = ax_g, ay_g, az_g
        self.prev_gx, self.prev_gy, self.prev_gz = gx, gy, gz

        # Mahony update expects accel vector (unitless direction OK)
        # we pass ax_g,ay_g,az_g in g-units (they will be normalized inside)
        self.mahony_update(ax_g, ay_g, az_g, gx, gy, gz, dt)

        # Prepare IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # linear acceleration in m/s^2
        imu_msg.linear_acceleration.x = ax_g * G_TO_M_S2
        imu_msg.linear_acceleration.y = ay_g * G_TO_M_S2
        imu_msg.linear_acceleration.z = az_g * G_TO_M_S2

        # angular velocity (rad/s)
        imu_msg.angular_velocity.x = gx
        imu_msg.angular_velocity.y = gy
        imu_msg.angular_velocity.z = gz

        # orientation quaternion
        imu_msg.orientation.w = self.q0
        imu_msg.orientation.x = self.q1
        imu_msg.orientation.y = self.q2
        imu_msg.orientation.z = self.q3

        # Covariances (set realistic non-zero values)
        # adjust these if you measure different noise levels
        imu_msg.orientation_covariance[0] = 0.002
        imu_msg.orientation_covariance[4] = 0.002
        imu_msg.orientation_covariance[8] = 0.002

        imu_msg.angular_velocity_covariance[0] = 0.02
        imu_msg.angular_velocity_covariance[4] = 0.02
        imu_msg.angular_velocity_covariance[8] = 0.02

        imu_msg.linear_acceleration_covariance[0] = 0.05
        imu_msg.linear_acceleration_covariance[4] = 0.05
        imu_msg.linear_acceleration_covariance[8] = 0.05

        # Publish
        self.pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
'''

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
CONFIG = 0x1A
ACCEL_XOUT = 0x3B
ACCEL_YOUT = 0x3D
ACCEL_ZOUT = 0x3F
GYRO_XOUT  = 0x43
GYRO_YOUT  = 0x45
GYRO_ZOUT  = 0x47

DEG2RAD = math.pi / 180.0
G_TO_M_S2 = 9.80665

def read_word(bus, addr, reg):
    high = bus.read_byte_data(addr, reg)
    low = bus.read_byte_data(addr, reg + 1)
    value = (high << 8) + low
    return value if value < 32768 else value - 65536


class ImuNode(Node):
    def __init__(self):
        super().__init__("imu_node")

        # Declare ROS2 parameters
        self.declare_parameter("gyro_bias_x", 0.0)
        self.declare_parameter("gyro_bias_y", 0.0)
        self.declare_parameter("gyro_bias_z", 0.0)
        self.declare_parameter("kp", 0.5)
        self.declare_parameter("ki", 0.0)
        self.declare_parameter("alpha", 0.6)
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("dlpf", 3)      # default: 44 Hz LPF
        
        # Load parameter values
       # self.load_parameters()
        self.gyro_bias_x = self.get_parameter("gyro_bias_x").get_parameter_value().double_value 
        self.gyro_bias_y = self.get_parameter("gyro_bias_y").get_parameter_value().double_value
        self.gyro_bias_z = self.get_parameter("gyro_bias_z").get_parameter_value().double_value 

        # I2C + MPU setup
        self.bus = smbus.SMBus(1)
        self.bus.write_byte_data(MPU_ADDR, PWR_MGMT, 0)  # wake
        self.set_dlpf(self.dlpf)

        # Publisher
        self.pub = self.create_publisher(Imu, "/imu/data", 50)

        # Filter state
        self.q0, self.q1, self.q2, self.q3 = 1, 0, 0, 0
        self.eInt = [0.0, 0.0, 0.0]

        # LPF previous values
        self.prev_ax = 0.0
        self.prev_ay = 0.0
        self.prev_az = 0.0
        self.prev_gx = 0.0
        self.prev_gy = 0.0
        self.prev_gz = 0.0

        # Gyro bias calibration
        self.gyro_bias = [0.0, 0.0, 0.0]
        self.calibrated = False
        self.cal_period = int(1.5 * self.rate_hz)
        self.cal_count = 0
        self.cal_sum = [0.0, 0.0, 0.0]

        # Timer
        self.timer = self.create_timer(1.0/self.rate_hz, self.loop)
        self.prev_time = time.monotonic()

        self.get_logger().info("IMU Mahony node with DLPF + Parameters started.")

    # ---------------- Parameter Loading ----------------
    def load_parameters(self):
        self.kp = float(self.get_parameter("kp").value)
        self.ki = float(self.get_parameter("ki").value)
        self.alpha = float(self.get_parameter("alpha").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.dlpf = int(self.get_parameter("dlpf").value)

    # ---------------- MPU6050 DLPF ---------------------
    def set_dlpf(self, dlpf_cfg):
        dlpf_cfg = max(0, min(6, dlpf_cfg))  # limit
        self.bus.write_byte_data(MPU_ADDR, CONFIG, dlpf_cfg)
        self.get_logger().info(f"MPU6050 DLPF set to mode {dlpf_cfg}")

    # ---------------- Main Loop ------------------------
    def loop(self):

        # Update parameters live
        self.load_parameters()

        dt_now = time.monotonic()
        dt = dt_now - self.prev_time
        self.prev_time = dt_now
        if dt <= 0:
            dt = 1.0/self.rate_hz

        # Read raw IMU
        ax = read_word(self.bus, MPU_ADDR, ACCEL_XOUT) / 16384.0
        ay = read_word(self.bus, MPU_ADDR, ACCEL_YOUT) / 16384.0
        az = read_word(self.bus, MPU_ADDR, ACCEL_ZOUT) / 16384.0

        gx = read_word(self.bus, MPU_ADDR, GYRO_XOUT) / 131.0
        gy = read_word(self.bus, MPU_ADDR, GYRO_YOUT) / 131.0
        gz = read_word(self.bus, MPU_ADDR, GYRO_ZOUT) / 131.0

        # Gyro bias calibration (first 1.5 seconds)
        if not self.calibrated:
            self.cal_sum[0] += gx
            self.cal_sum[1] += gy
            self.cal_sum[2] += gz
            self.cal_count += 1
            if self.cal_count >= self.cal_period:
                self.gyro_bias = [
                    self.cal_sum[0]/self.cal_count,
                    self.cal_sum[1]/self.cal_count,
                    self.cal_sum[2]/self.cal_count,
                ]
                self.calibrated = True
                self.get_logger().info(
                    f"Gyro calibrated: bias = {self.gyro_bias}"
                )
            return

        # Remove gyro bias
        gx -= self.gyro_bias[0]
        gy -= self.gyro_bias[1]
        gz -= self.gyro_bias[2]

        # Convert to rad/s
        gx *= DEG2RAD
        gy *= DEG2RAD
        gz *= DEG2RAD

        # LPF smoothing
        ax = self.alpha * ax + (1 - self.alpha) * self.prev_ax
        ay = self.alpha * ay + (1 - self.alpha) * self.prev_ay
        az = self.alpha * az + (1 - self.alpha) * self.prev_az
        self.prev_ax, self.prev_ay, self.prev_az = ax, ay, az

        gx = self.alpha * gx + (1 - self.alpha) * self.prev_gx
        gy = self.alpha * gy + (1 - self.alpha) * self.prev_gy
        gz = self.alpha * gz + (1 - self.alpha) * self.prev_gz
        self.prev_gx, self.prev_gy, self.prev_gz = gx, gy, gz

        # Normalize accel
        norm = math.sqrt(ax*ax + ay*ay + az*az)
        if norm == 0:
            return
        ax /= norm
        ay /= norm
        az /= norm

        # Estimated gravity from quaternion
        v_x = 2*(self.q1*self.q3 - self.q0*self.q2)
        v_y = 2*(self.q0*self.q1 + self.q2*self.q3)
        v_z = self.q0*self.q0 - self.q1*self.q1 - self.q2*self.q2 + self.q3*self.q3

        # Error = a × v
        ex = ay * v_z - az * v_y
        ey = az * v_x - ax * v_z
        ez = ax * v_y - ay * v_x

        # Integral error
        self.eInt[0] += ex * self.ki * dt
        self.eInt[1] += ey * self.ki * dt
        self.eInt[2] += ez * self.ki * dt

        # Apply feedback
        gx += self.kp * ex + self.eInt[0]
        gy += self.kp * ey + self.eInt[1]
        gz += self.kp * ez + self.eInt[2]

        # Quaternion update
        q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3

        q0 += 0.5 * (-q1*gx - q2*gy - q3*gz) * dt
        q1 += 0.5 * (q0*gx + q2*gz - q3*gy) * dt
        q2 += 0.5 * (q0*gy - q1*gz + q3*gx) * dt
        q3 += 0.5 * (q0*gz + q1*gy - q2*gx) * dt

        # Normalize quaternion
        norm_q = math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
        q0 /= norm_q
        q1 /= norm_q
        q2 /= norm_q
        q3 /= norm_q

        self.q0, self.q1, self.q2, self.q3 = q0, q1, q2, q3

        # Publish IMU message
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        # Orientation
        msg.orientation.w = q0
        msg.orientation.x = q1
        msg.orientation.y = q2
        msg.orientation.z = q3

        # Gyro (rad/s)
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        # Accel (m/s^2)
        msg.linear_acceleration.x = ax * G_TO_M_S2
        msg.linear_acceleration.y = ay * G_TO_M_S2
        msg.linear_acceleration.z = az * G_TO_M_S2

        # Covariances (tune for EKF)
        msg.orientation_covariance = [0.0025, 0, 0,
                                      0, 0.0025, 0,
                                      0, 0, 0.0025]
        msg.angular_velocity_covariance = [0.01, 0, 0,
                                           0, 0.01, 0,
                                           0, 0, 0.01]
        msg.linear_acceleration_covariance = [0.04, 0, 0,
                                              0, 0.04, 0,
                                              0, 0, 0.04]

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

