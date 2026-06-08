"""Real-Tello driver: bridges djitellopy <-> AutoBee /drone/* topic graph.

Subscribes:
    /drone/cmd_vel    geometry_msgs/Twist  -> tello.send_rc_control(...)
    /mission_status   std_msgs/String JSON -> takeoff()/land() on lifecycle events
    /mission_abort    std_msgs/String      -> emergency land()

Publishes:
    /drone/camera/image_raw  sensor_msgs/Image  (BGR8, ~10 Hz)
    /drone/imu               sensor_msgs/Imu    (orientation + linear accel, ~10 Hz)

Designed to be a 1:1 swap for autobee_sim. The rest of the topic graph
is unchanged, so demo_tello.launch.py reuses every other node verbatim.
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import String

import cv2  # noqa: F401  -- pulled in via cv_bridge
from cv_bridge import CvBridge
from djitellopy import Tello

GRAVITY = 9.80665


def _euler_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5),  math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5),   math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,  # x
        cr * sp * cy + sr * cp * sy,  # y
        cr * cp * sy - sr * sp * cy,  # z
        cr * cp * cy + sr * sp * sy,  # w
    )


def _clamp_int(v, lo=-100, hi=100):
    return int(max(lo, min(hi, v)))


class TelloDriver(Node):
    def __init__(self):
        super().__init__("tello_driver_node")

        self.declare_parameter("linear_max_mps", 1.0)
        self.declare_parameter("angular_max_rps", 1.0)
        self.declare_parameter("camera_fps", 10.0)
        self.declare_parameter("imu_hz", 10.0)
        # State drives /drone/odom dead-reckoning + closed-loop feedback, so it
        # must run fast enough to integrate (1 Hz was fine for a status HUD but
        # far too coarse for control). The Tello broadcasts state at ~10 Hz.
        self.declare_parameter("state_hz", 10.0)
        self.declare_parameter("min_battery_pct", 25)
        self.declare_parameter("cmd_vel_timeout_s", 1.0)
        self.declare_parameter("auto_takeoff_on_first_cmd", True)

        self.linear_max = float(self.get_parameter("linear_max_mps").value)
        self.angular_max = float(self.get_parameter("angular_max_rps").value)
        cam_fps = float(self.get_parameter("camera_fps").value)
        imu_hz = float(self.get_parameter("imu_hz").value)
        state_hz = float(self.get_parameter("state_hz").value)
        min_batt = int(self.get_parameter("min_battery_pct").value)
        self.cmd_vel_timeout = float(self.get_parameter("cmd_vel_timeout_s").value)
        self.auto_takeoff = bool(self.get_parameter("auto_takeoff_on_first_cmd").value)

        self.get_logger().info("connecting to Tello...")
        self.tello = Tello()
        self.tello.connect()
        battery = self.tello.get_battery()
        self.get_logger().info(f"Tello connected. battery={battery}%")
        if battery < min_batt:
            raise RuntimeError(
                f"battery {battery}% < min_battery_pct {min_batt}%; charge before flying"
            )

        self.tello.streamon()
        self.frame_read = self.tello.get_frame_read()
        time.sleep(1.0)

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, "/drone/camera/image_raw", 10)
        self.imu_pub = self.create_publisher(Imu, "/drone/imu", 10)
        self.state_pub = self.create_publisher(String, "/drone/state", 10)

        self.create_subscription(Twist, "/drone/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(String, "/mission_status", self._on_mission_status, 10)
        self.create_subscription(String, "/mission_abort", self._on_abort, 10)

        self.create_timer(1.0 / cam_fps, self._publish_frame)
        self.create_timer(1.0 / imu_hz, self._publish_imu)
        self.create_timer(0.1, self._safety_tick)
        self.create_timer(1.0 / state_hz, self._publish_state)

        self._airborne = False
        self._last_cmd_time = 0.0
        self._last_nonzero_cmd = 0.0

        self.get_logger().info(
            f"tello_driver_node up. "
            f"linear_max={self.linear_max}m/s angular_max={self.angular_max}rad/s "
            f"auto_takeoff={self.auto_takeoff}"
        )

    def _publish_frame(self):
        f = self.frame_read.frame
        if f is None:
            return
        msg = self.bridge.cv2_to_imgmsg(f, "bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "drone/camera"
        self.image_pub.publish(msg)

    def _publish_imu(self):
        try:
            s = self.tello.get_current_state()
        except Exception:
            return
        if not s:
            return
        roll = math.radians(float(s.get("roll", 0)))
        pitch = math.radians(float(s.get("pitch", 0)))
        yaw = math.radians(float(s.get("yaw", 0)))
        qx, qy, qz, qw = _euler_to_quat(roll, pitch, yaw)
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "drone/imu"
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = qx, qy, qz, qw
        msg.linear_acceleration.x = float(s.get("agx", 0)) * 0.001 * GRAVITY
        msg.linear_acceleration.y = float(s.get("agy", 0)) * 0.001 * GRAVITY
        msg.linear_acceleration.z = float(s.get("agz", 0)) * 0.001 * GRAVITY
        self.imu_pub.publish(msg)

    def _publish_state(self):
        try:
            s = self.tello.get_current_state()
        except Exception:
            return
        if not s:
            return
        snapshot = {
            "battery_pct":  s.get("bat"),
            "temperature_c": s.get("temph"),
            "height_cm":    s.get("h"),
            "tof_cm":       s.get("tof"),
            "barometer_cm": s.get("baro"),
            "pitch_deg":    s.get("pitch"),
            "roll_deg":     s.get("roll"),
            "yaw_deg":      s.get("yaw"),
            "vx_cm_s":      s.get("vgx"),
            "vy_cm_s":      s.get("vgy"),
            "vz_cm_s":      s.get("vgz"),
            "flight_time_s": s.get("time"),
            "airborne":     self._airborne,
        }
        self.state_pub.publish(String(data=json.dumps(snapshot)))

    def _on_cmd_vel(self, msg: Twist):
        nonzero = any(abs(v) > 1e-3 for v in (
            msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z
        ))
        now = time.time()
        self._last_cmd_time = now
        if nonzero:
            self._last_nonzero_cmd = now

        if not self._airborne:
            if self.auto_takeoff and nonzero:
                self.get_logger().info("first non-zero cmd_vel -> takeoff()")
                try:
                    self.tello.takeoff()
                    self._airborne = True
                except Exception as e:
                    self.get_logger().error(f"takeoff failed: {e}")
                    return
            else:
                return

        # ROS REP-103: x=forward, y=left, z=up; angular.z=ccw-positive yaw.
        # Tello rc: lr (right+), fb (forward+), ud (up+), yaw (cw+).
        fb  = _clamp_int( msg.linear.x  / self.linear_max  * 100)
        lr  = _clamp_int(-msg.linear.y  / self.linear_max  * 100)
        ud  = _clamp_int( msg.linear.z  / self.linear_max  * 100)
        yaw = _clamp_int(-msg.angular.z / self.angular_max * 100)
        self.tello.send_rc_control(lr, fb, ud, yaw)

    def _safety_tick(self):
        if not self._airborne:
            return
        if time.time() - self._last_cmd_time > self.cmd_vel_timeout:
            self.tello.send_rc_control(0, 0, 0, 0)

    def _on_mission_status(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        event = str(data.get("event", ""))
        if event in ("completed", "aborted", "failed") and self._airborne:
            self.get_logger().info(f"/mission_status {event} -> land()")
            self._land_safely()

    def _on_abort(self, _msg: String):
        if self._airborne:
            self.get_logger().warn("/mission_abort -> land()")
            self._land_safely()

    def _land_safely(self):
        try:
            self.tello.send_rc_control(0, 0, 0, 0)
            self.tello.land()
        except Exception as e:
            self.get_logger().error(f"land() raised {e}; falling back to emergency()")
            try:
                self.tello.emergency()
            except Exception:
                pass
        finally:
            self._airborne = False

    def destroy_node(self):
        try:
            if self._airborne:
                self._land_safely()
            self.tello.streamoff()
            self.tello.end()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = TelloDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
