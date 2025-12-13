#!/usr/bin/env python3
'''
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus
import math
import time
from rclpy.parameter import Parameter

# MPU6050 Registers
MPU_ADDR   = 0x68
PWR_MGMT   = 0x6B
CONFIG     = 0x1A
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
    low  = bus.read_byte_data(addr, reg + 1)
    value = (high << 8) | low
    return value if value < 32768 else value - 65536


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        # ---------------- Declare parameters ----------------
        self.declare_parameter('auto_calibrate', True)
        self.declare_parameter('calibration_duration', 1.5)   # seconds
        self.declare_parameter('gyro_bias_x', 0.0)
        self.declare_parameter('gyro_bias_y', 0.0)
        self.declare_parameter('gyro_bias_z', 0.0)

        self.declare_parameter('kp', 0.5)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('alpha', 0.6)      # LPF smoothing (0..1)
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('dlpf', 3)         # MPU DLPF mode (0..6)

        # Load parameters (initial)
        self.load_parameters()

        # ---------------- I2C / MPU setup ----------------
        try:
            self.bus = smbus.SMBus(1)
            self.bus.write_byte_data(MPU_ADDR, PWR_MGMT, 0)  # wake device
            self.set_dlpf(self.dlpf)
        except Exception as e:
            self.get_logger().error(f"I2C/MPU init error: {e}")
            raise

        # ---------------- Publisher & state ----------------
        self.pub = self.create_publisher(Imu, '/imu/data', 50)

        # Mahony quaternion state (q0 = w)
        self.q0 = 1.0
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0
        self.eInt = [0.0, 0.0, 0.0]

        # LPF previous values
        self.prev_ax = 0.0
        self.prev_ay = 0.0
        self.prev_az = 0.0
        self.prev_gx = 0.0
        self.prev_gy = 0.0
        self.prev_gz = 0.0

        # Calibration accumulators (calibration performed in deg/s units)
        self.calibrated = False
        self.cal_sum = [0.0, 0.0, 0.0]
        self.cal_count = 0
        self.cal_samples = max(1, int(self.calibration_duration * self.rate_hz))

        # Stored bias parameters (user-provided)
        self.gyro_bias_param = [self.gyro_bias_x, self.gyro_bias_y, self.gyro_bias_z]
        # Combined bias = auto_calculated + param offset (in deg/s)
        self.gyro_bias_auto = [0.0, 0.0, 0.0]

        # Timing
        self.prev_time = time.monotonic()
        self.timer = self.create_timer(1.0 / self.rate_hz, self.loop)

        self.get_logger().info('IMU Mahony node started. auto_calibrate=%s, cal_samples=%d' %
                               (str(self.auto_calibrate), self.cal_samples))

    # ---------------- Parameter loader ----------------
    def load_parameters(self):
        # Read params (safe retrieval)
        self.auto_calibrate = bool(self.get_parameter('auto_calibrate').value)
        self.calibration_duration = float(self.get_parameter('calibration_duration').value)

        self.gyro_bias_x = float(self.get_parameter('gyro_bias_x').value)
        self.gyro_bias_y = float(self.get_parameter('gyro_bias_y').value)
        self.gyro_bias_z = float(self.get_parameter('gyro_bias_z').value)
        self.gyro_bias_param = [self.gyro_bias_x, self.gyro_bias_y, self.gyro_bias_z]

        self.kp = float(self.get_parameter('kp').value)
        self.ki = float(self.get_parameter('ki').value)
        self.alpha = float(self.get_parameter('alpha').value)
        if self.alpha < 0.0:
            self.alpha = 0.0
        if self.alpha > 1.0:
            self.alpha = 1.0

        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.dlpf = int(self.get_parameter('dlpf').value)

        # update cal_samples if calibration_duration or rate changed
        self.cal_samples = max(1, int(self.calibration_duration * self.rate_hz))

    # ---------------- MPU6050 DLPF ----------------
    def set_dlpf(self, dlpf_cfg):
        dlpf_cfg = max(0, min(6, int(dlpf_cfg)))
        try:
            self.bus.write_byte_data(MPU_ADDR, CONFIG, dlpf_cfg)
            self.get_logger().info(f"MPU6050 DLPF set to mode {dlpf_cfg}")
        except Exception as e:
            self.get_logger().warn(f"Failed to set DLPF: {e}")

    # ---------------- Mahony update ----------------
    def mahony_update(self, gx, gy, gz, ax, ay, az, dt):
        # gx,gy,gz are in rad/s
        # ax,ay,az are in g units (will be normalized)
        # Normalize accel
        norm = math.sqrt(ax*ax + ay*ay + az*az)
        if norm < 1e-6:
            return
        axn = ax / norm
        ayn = ay / norm
        azn = az / norm

        # estimated gravity direction from quaternion
        vx = 2.0 * (self.q1 * self.q3 - self.q0 * self.q2)
        vy = 2.0 * (self.q0 * self.q1 + self.q2 * self.q3)
        vz = self.q0*self.q0 - self.q1*self.q1 - self.q2*self.q2 + self.q3*self.q3

        # error (cross product)
        ex = (ayn * vz - azn * vy)
        ey = (azn * vx - axn * vz)
        ez = (axn * vy - ayn * vx)

        # integral
        if self.ki > 0.0:
            self.eInt[0] += ex * dt
            self.eInt[1] += ey * dt
            self.eInt[2] += ez * dt
        else:
            # avoid integral windup if ki == 0
            self.eInt = [0.0, 0.0, 0.0]

        # apply PI feedback to gyro (gx,gy,gz already in rad/s)
        gx += self.kp * ex + self.ki * self.eInt[0]
        gy += self.kp * ey + self.ki * self.eInt[1]
        gz += self.kp * ez + self.ki * self.eInt[2]

        # quaternion derivative integration
        qDot0 = 0.5 * (-self.q1 * gx - self.q2 * gy - self.q3 * gz)
        qDot1 = 0.5 * (self.q0 * gx + self.q2 * gz - self.q3 * gy)
        qDot2 = 0.5 * (self.q0 * gy - self.q1 * gz + self.q3 * gx)
        qDot3 = 0.5 * (self.q0 * gz + self.q1 * gy - self.q2 * gx)

        self.q0 += qDot0 * dt
        self.q1 += qDot1 * dt
        self.q2 += qDot2 * dt
        self.q3 += qDot3 * dt

        # normalize quaternion
        qnorm = math.sqrt(self.q0*self.q0 + self.q1*self.q1 + self.q2*self.q2 + self.q3*self.q3)
        if qnorm > 0:
            inv = 1.0 / qnorm
            self.q0 *= inv
            self.q1 *= inv
            self.q2 *= inv
            self.q3 *= inv

    # ---------------- main loop ----------------
    def loop(self):
        # reload parameters every loop (allows dynamic tuning)
        self.load_parameters()

        # timing
        now = time.monotonic()
        dt = now - self.prev_time
        if dt <= 0:
            dt = 1.0 / max(1.0, self.rate_hz)
        self.prev_time = now

        # read raw values
        try:
            raw_ax = read_word(self.bus, MPU_ADDR, ACCEL_XOUT)
            raw_ay = read_word(self.bus, MPU_ADDR, ACCEL_YOUT)
            raw_az = read_word(self.bus, MPU_ADDR, ACCEL_ZOUT)
            raw_gx = read_word(self.bus, MPU_ADDR, GYRO_XOUT)
            raw_gy = read_word(self.bus, MPU_ADDR, GYRO_YOUT)
            raw_gz = read_word(self.bus, MPU_ADDR, GYRO_ZOUT)
        except Exception as e:
            self.get_logger().error(f"I2C read error: {e}")
            return

        # convert to physical units
        ax = raw_ax / 16384.0   # in g
        ay = raw_ay / 16384.0
        az = raw_az / 16384.0

        gx_deg = raw_gx / 131.0  # in deg/s (typical full-scale ±250°/s)
        gy_deg = raw_gy / 131.0
        gz_deg = raw_gz / 131.0

        # ---------------- Calibration phase (deg/s) ----------------
        if self.auto_calibrate and not self.calibrated:
            self.cal_sum[0] += gx_deg
            self.cal_sum[1] += gy_deg
            self.cal_sum[2] += gz_deg
            self.cal_count += 1
            if self.cal_count >= self.cal_samples:
                self.gyro_bias_auto = [
                    self.cal_sum[0] / self.cal_count,
                    self.cal_sum[1] / self.cal_count,
                    self.cal_sum[2] / self.cal_count
                ]
                self.calibrated = True
                # log and instruct user to paste these into YAML if desired
                self.get_logger().info(
                    "Gyro auto-calibrated (deg/s): [%0.6f, %0.6f, %0.6f]. "
                    "You may copy these into your imu_params.yaml as gyro_bias_x/y/z and set auto_calibrate=false."
                    % tuple(self.gyro_bias_auto)
                )
            return

        # Combined bias: auto_calculated + user parameter (deg/s)
        bias_x = self.gyro_bias_auto[0] + self.gyro_bias_param[0]
        bias_y = self.gyro_bias_auto[1] + self.gyro_bias_param[1]
        bias_z = self.gyro_bias_auto[2] + self.gyro_bias_param[2]

        # Remove bias (still in deg/s), then convert to rad/s
        gx = (gx_deg - bias_x) * DEG2RAD
        gy = (gy_deg - bias_y) * DEG2RAD
        gz = (gz_deg - bias_z) * DEG2RAD

        # Low-pass filter (exponential) on accel (g) and gyro (rad/s)
        alpha = self.alpha
        ax = alpha * ax + (1.0 - alpha) * self.prev_ax
        ay = alpha * ay + (1.0 - alpha) * self.prev_ay
        az = alpha * az + (1.0 - alpha) * self.prev_az
        self.prev_ax, self.prev_ay, self.prev_az = ax, ay, az

        gx = alpha * gx + (1.0 - alpha) * self.prev_gx
        gy = alpha * gy + (1.0 - alpha) * self.prev_gy
        gz = alpha * gz + (1.0 - alpha) * self.prev_gz
        self.prev_gx, self.prev_gy, self.prev_gz = gx, gy, gz

        # Mahony filter update (expects accel in g, gyro in rad/s)
        self.mahony_update(gx, gy, gz, ax, ay, az, dt)

        # Build IMU message
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Orientation (quaternion)
        msg.orientation.w = self.q0
        msg.orientation.x = self.q1
        msg.orientation.y = self.q2
        msg.orientation.z = self.q3

        # Angular velocity (rad/s)
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        # Linear acceleration (m/s^2)
        msg.linear_acceleration.x = ax * G_TO_M_S2
        msg.linear_acceleration.y = ay * G_TO_M_S2
        msg.linear_acceleration.z = az * G_TO_M_S2

        # Covariances (diagonal variances; tune as needed)
        msg.orientation_covariance = [
            0.0025, 0.0, 0.0,
            0.0, 0.0025, 0.0,
            0.0, 0.0, 0.0025
        ]
        msg.angular_velocity_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]
        msg.linear_acceleration_covariance = [
            0.04, 0.0, 0.0,
            0.0, 0.04, 0.0,
            0.0, 0.0, 0.04
        ]

        # Publish
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
'''

#!/usr/bin/env python3
"""
Production-ready ROS2 IMU node with Mahony filter for MPU6050.

Features:
- DLPF configuration (MPU CONFIG register)
- Auto calibration (optional) or use static gyro_bias from parameters
- Live-tunable kp, ki, alpha via ros2 param set
- Publishes sensor_msgs/Imu with covariances suitable for EKF
- Lightweight exponential LPF for accel+gyro
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus
import math
import time

# MPU6050 Registers
MPU_ADDR   = 0x68
PWR_MGMT   = 0x6B
CONFIG     = 0x1A
ACCEL_XOUT = 0x3B
ACCEL_YOUT = 0x3D
ACCEL_ZOUT = 0x3F
GYRO_XOUT  = 0x43
GYRO_YOUT  = 0x45
GYRO_ZOUT  = 0x47

DEG2RAD = math.pi / 180.0
G_TO_M_S2 = 9.80665


def read_word(bus, addr, reg):
    """Read a signed 16-bit value from I2C (big-endian high/low)."""
    high = bus.read_byte_data(addr, reg)
    low = bus.read_byte_data(addr, reg + 1)
    value = (high << 8) | low
    return value if value < 32768 else value - 65536


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        # ---------------- Declare parameters ----------------
        # Calibration / biases
        self.declare_parameter('auto_calibrate', True)
        self.declare_parameter('calibration_duration', 1.5)   # seconds
        self.declare_parameter('gyro_bias_x', 0.0)            # deg/s
        self.declare_parameter('gyro_bias_y', 0.0)            # deg/s
        self.declare_parameter('gyro_bias_z', 0.0)            # deg/s

        # Mahony and filtering
        self.declare_parameter('kp', 0.5)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('alpha', 0.6)      # exponential smoothing (0..1)
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('dlpf', 3)         # MPU6050 DLPF mode (0..6)

        # Covariances (defaults; can be overridden in YAML)
        self.declare_parameter('orientation_covariance', [0.0025, 0.0, 0.0,
                                                          0.0, 0.0025, 0.0,
                                                          0.0, 0.0, 0.0025])
        self.declare_parameter('angular_velocity_covariance', [0.01, 0.0, 0.0,
                                                                0.0, 0.01, 0.0,
                                                                0.0, 0.0, 0.01])
        self.declare_parameter('linear_acceleration_covariance', [0.04, 0.0, 0.0,
                                                                  0.0, 0.04, 0.0,
                                                                  0.0, 0.0, 0.04])

        # Load initial parameters
        self.load_parameters(initial=True)

        # ---------------- Initialize I2C and MPU ----------------
        try:
            self.bus = smbus.SMBus(1)
            self.bus.write_byte_data(MPU_ADDR, PWR_MGMT, 0)  # wake device
            self.set_dlpf(self.dlpf)
        except Exception as e:
            self.get_logger().error(f"I2C/MPU initialization failed: {e}")
            raise

        # ---------------- Publisher & internal state ----------------
        self.pub = self.create_publisher(Imu, '/imu/data', 50)

        # Quaternion state (q0 = w)
        self.q0 = 1.0
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0
        self.eInt = [0.0, 0.0, 0.0]

        # LPF previous values
        self.prev_ax = 0.0
        self.prev_ay = 0.0
        self.prev_az = 0.0
        self.prev_gx = 0.0
        self.prev_gy = 0.0
        self.prev_gz = 0.0

        # Calibration accumulators (deg/s units)
        self.calibrated = False
        self.cal_sum = [0.0, 0.0, 0.0]
        self.cal_count = 0
        self.cal_samples = max(1, int(self.calibration_duration * self.rate_hz))

        # Parameter-provided bias (deg/s). Combined bias = auto + param
        self.gyro_bias_param = [self.gyro_bias_x, self.gyro_bias_y, self.gyro_bias_z]
        self.gyro_bias_auto = [0.0, 0.0, 0.0]

        # Timing and timer
        self.prev_time = time.monotonic()
        self.timer = self.create_timer(1.0 / self.rate_hz, self.loop)

        self.get_logger().info(f"IMU node started (auto_calibrate={self.auto_calibrate}, "
                               f"cal_samples={self.cal_samples}, rate_hz={self.rate_hz})")

        # If auto_calibrate disabled, mark calibrated and use static bias
        if not self.auto_calibrate:
            self.gyro_bias_auto = [0.0, 0.0, 0.0]
            self.calibrated = True
            self.get_logger().info(f"Auto calibration disabled; using static bias (deg/s): {self.gyro_bias_param}")

    # ---------------- Parameter loader ----------------
    def load_parameters(self, initial=False):
        """Load parameters from ROS2 parameter server. If parameters are changed at runtime,
        this function reads them. Note: changing rate_hz requires node restart to take effect."""
        # Read params (safe retrieval)
        try:
            self.auto_calibrate = bool(self.get_parameter('auto_calibrate').value)
        except Exception:
            self.auto_calibrate = True

        try:
            self.calibration_duration = float(self.get_parameter('calibration_duration').value)
        except Exception:
            self.calibration_duration = 1.5

        # static bias params (deg/s)
        self.gyro_bias_x = float(self.get_parameter('gyro_bias_x').value)
        self.gyro_bias_y = float(self.get_parameter('gyro_bias_y').value)
        self.gyro_bias_z = float(self.get_parameter('gyro_bias_z').value)
        self.gyro_bias_param = [self.gyro_bias_x, self.gyro_bias_y, self.gyro_bias_z]

        # Mahony & filter
        self.kp = float(self.get_parameter('kp').value)
        self.ki = float(self.get_parameter('ki').value)
        self.alpha = float(self.get_parameter('alpha').value)
        if self.alpha < 0.0:
            self.alpha = 0.0
        if self.alpha > 1.0:
            self.alpha = 1.0

        # rate and dlpf
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.dlpf = int(self.get_parameter('dlpf').value)

        # Covariances
        try:
            self.orientation_cov = self.get_parameter('orientation_covariance').value
            self.angular_vel_cov = self.get_parameter('angular_velocity_covariance').value
            self.accel_cov = self.get_parameter('linear_acceleration_covariance').value
        except Exception:
            # fallback to defaults previously declared
            self.orientation_cov = [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025]
            self.angular_vel_cov = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
            self.accel_cov = [0.04, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.04]

        # update cal_samples if changed
        self.cal_samples = max(1, int(self.calibration_duration * self.rate_hz))

        # If user disables auto_calibrate at runtime, ensure we mark calibrated and use param bias
        if not self.auto_calibrate and not self.calibrated:
            self.gyro_bias_auto = [0.0, 0.0, 0.0]
            self.calibrated = False
            self.get_logger().info("Auto calibration disabled at runtime; using static bias from params.")

        # If initial call and auto_calibrate is False, ensure cal state set
        if initial and not self.auto_calibrate:
            self.gyro_bias_auto = [0.0, 0.0, 0.0]
            self.calibrated = True

    # ---------------- MPU6050 DLPF ----------------
    def set_dlpf(self, dlpf_cfg):
        dlpf_cfg = max(0, min(6, int(dlpf_cfg)))
        try:
            self.bus.write_byte_data(MPU_ADDR, CONFIG, dlpf_cfg)
            self.get_logger().info(f"MPU6050 DLPF set to mode {dlpf_cfg}")
        except Exception as e:
            self.get_logger().warning(f"Failed to set DLPF: {e}")

    # ---------------- Mahony update ----------------
    def mahony_update(self, gx, gy, gz, ax, ay, az, dt):
        # gx, gy, gz in rad/s; ax, ay, az in g
        norm = math.sqrt(ax*ax + ay*ay + az*az)
        if norm < 1e-6:
            return
        axn = ax / norm
        ayn = ay / norm
        azn = az / norm

        # estimated gravity direction from quaternion
        vx = 2.0 * (self.q1 * self.q3 - self.q0 * self.q2)
        vy = 2.0 * (self.q0 * self.q1 + self.q2 * self.q3)
        vz = self.q0*self.q0 - self.q1*self.q1 - self.q2*self.q2 + self.q3*self.q3

        # error = accel × gravity_est
        ex = (ayn * vz - azn * vy)
        ey = (azn * vx - axn * vz)
        ez = (axn * vy - ayn * vx)

        # integral
        if self.ki > 0.0:
            self.eInt[0] += ex * dt
            self.eInt[1] += ey * dt
            self.eInt[2] += ez * dt
        else:
            # reset integral to avoid wind-up
            self.eInt = [0.0, 0.0, 0.0]

        # apply PI feedback
        gx += self.kp * ex + self.ki * self.eInt[0]
        gy += self.kp * ey + self.ki * self.eInt[1]
        gz += self.kp * ez + self.ki * self.eInt[2]

        # quaternion derivative and integration
        qDot0 = 0.5 * (-self.q1 * gx - self.q2 * gy - self.q3 * gz)
        qDot1 = 0.5 * (self.q0 * gx + self.q2 * gz - self.q3 * gy)
        qDot2 = 0.5 * (self.q0 * gy - self.q1 * gz + self.q3 * gx)
        qDot3 = 0.5 * (self.q0 * gz + self.q1 * gy - self.q2 * gx)

        self.q0 += qDot0 * dt
        self.q1 += qDot1 * dt
        self.q2 += qDot2 * dt
        self.q3 += qDot3 * dt

        # normalize
        qnorm = math.sqrt(self.q0*self.q0 + self.q1*self.q1 + self.q2*self.q2 + self.q3*self.q3)
        if qnorm > 0.0:
            inv = 1.0 / qnorm
            self.q0 *= inv
            self.q1 *= inv
            self.q2 *= inv
            self.q3 *= inv

    # ---------------- main loop ----------------
    def loop(self):
        # reload params (allows dynamic tuning of gains)
        self.load_parameters()

        # timing
        now = time.monotonic()
        dt = now - self.prev_time
        if dt <= 0:
            dt = 1.0 / max(1.0, self.rate_hz)
        self.prev_time = now

        # read raw registers
        try:
            raw_ax = read_word(self.bus, MPU_ADDR, ACCEL_XOUT)
            raw_ay = read_word(self.bus, MPU_ADDR, ACCEL_YOUT)
            raw_az = read_word(self.bus, MPU_ADDR, ACCEL_ZOUT)
            raw_gx = read_word(self.bus, MPU_ADDR, GYRO_XOUT)
            raw_gy = read_word(self.bus, MPU_ADDR, GYRO_YOUT)
            raw_gz = read_word(self.bus, MPU_ADDR, GYRO_ZOUT)
        except Exception as e:
            self.get_logger().error(f"I2C read error: {e}")
            return

        # convert units
        ax = raw_ax / 16384.0   # g
        ay = raw_ay / 16384.0
        az = raw_az / 16384.0

        gx_deg = raw_gx / 131.0  # deg/s
        gy_deg = raw_gy / 131.0
        gz_deg = raw_gz / 131.0

        # ---------------- calibration logic ----------------
        # Only auto-calibrate if enabled
        if self.auto_calibrate and not self.calibrated:
            self.cal_sum[0] += gx_deg
            self.cal_sum[1] += gy_deg
            self.cal_sum[2] += gz_deg
            self.cal_count += 1
            if self.cal_count >= self.cal_samples:
                self.gyro_bias_auto = [
                    self.cal_sum[0] / self.cal_count,
                    self.cal_sum[1] / self.cal_count,
                    self.cal_sum[2] / self.cal_count
                ]
                self.calibrated = True
                self.get_logger().info(
                    "Gyro auto-calibrated (deg/s): [%0.6f, %0.6f, %0.6f]. "
                    "To skip calibration next run, copy values into gyro_bias_x/y/z and set auto_calibrate=false."
                    % tuple(self.gyro_bias_auto)
                )
            return
        # If auto_calibrate disabled but not yet set calibrated flag, set it (use param bias)
        if not self.auto_calibrate and not self.calibrated:
            self.gyro_bias_auto = [0.0, 0.0, 0.0]
            self.calibrated = True
            self.get_logger().info("Auto calibration disabled; using static bias from parameters.")

        # Combined bias (deg/s)
        bias_x = self.gyro_bias_auto[0] + self.gyro_bias_param[0]
        bias_y = self.gyro_bias_auto[1] + self.gyro_bias_param[1]
        bias_z = self.gyro_bias_auto[2] + self.gyro_bias_param[2]

        # subtract bias and convert to rad/s
        gx = (gx_deg - bias_x) * DEG2RAD
        gy = (gy_deg - bias_y) * DEG2RAD
        gz = (gz_deg - bias_z) * DEG2RAD

        # LPF (exponential)
        a = self.alpha
        ax = a * ax + (1.0 - a) * self.prev_ax
        ay = a * ay + (1.0 - a) * self.prev_ay
        az = a * az + (1.0 - a) * self.prev_az
        self.prev_ax, self.prev_ay, self.prev_az = ax, ay, az

        gx = a * gx + (1.0 - a) * self.prev_gx
        gy = a * gy + (1.0 - a) * self.prev_gy
        gz = a * gz + (1.0 - a) * self.prev_gz
        self.prev_gx, self.prev_gy, self.prev_gz = gx, gy, gz

        # run Mahony (accel in g, gyro in rad/s)
        self.mahony_update(gx, gy, gz, ax, ay, az, dt)

        # compose IMU message
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        msg.orientation.w = self.q0
        msg.orientation.x = self.q1
        msg.orientation.y = self.q2
        msg.orientation.z = self.q3

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        msg.linear_acceleration.x = ax * G_TO_M_S2
        msg.linear_acceleration.y = ay * G_TO_M_S2
        msg.linear_acceleration.z = az * G_TO_M_S2

        # covariances from parameters (9-element arrays)
        msg.orientation_covariance = list(self.orientation_cov)
        msg.angular_velocity_covariance = list(self.angular_vel_cov)
        msg.linear_acceleration_covariance = list(self.accel_cov)

        # publish
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

