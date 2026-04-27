import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState


class CmdVelToJointStates(Node):
    def __init__(self) -> None:
        super().__init__('cmd_vel_to_joint_states')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('wheelbase', 0.215)
        self.declare_parameter('track_width', 0.16)
        self.declare_parameter('wheel_radius', 0.0325)
        self.declare_parameter('max_steering_angle', 0.52)

        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        joint_states_topic = str(self.get_parameter('joint_states_topic').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.track_width = float(self.get_parameter('track_width').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.max_steering_angle = float(self.get_parameter('max_steering_angle').value)

        self.linear_v = 0.0
        self.yaw_rate = 0.0

        self.front_left_wheel_pos = 0.0
        self.front_right_wheel_pos = 0.0
        self.rear_left_wheel_pos = 0.0
        self.rear_right_wheel_pos = 0.0
        self.last_t = self.get_clock().now()

        self.publisher = self.create_publisher(JointState, joint_states_topic, 1)
        self.create_subscription(Twist, cmd_vel_topic, self.cmd_vel_callback, 1)
        self.create_timer(1.0 / max(self.publish_rate_hz, 1.0), self.publish_joint_states)

    def cmd_vel_callback(self, msg: Twist) -> None:
        self.linear_v = float(msg.linear.x)
        self.yaw_rate = float(msg.angular.z)

    def publish_joint_states(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_t).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self.last_t = now

        steering_left, steering_right = self._compute_steering(self.linear_v, self.yaw_rate)
        rear_left_vel, rear_right_vel = self._compute_wheel_angular_vel(self.linear_v, self.yaw_rate)

        # Front wheel spin approximated from local path speeds.
        front_left_vel = rear_left_vel
        front_right_vel = rear_right_vel

        self.front_left_wheel_pos += front_left_vel * dt
        self.front_right_wheel_pos += front_right_vel * dt
        self.rear_left_wheel_pos += rear_left_vel * dt
        self.rear_right_wheel_pos += rear_right_vel * dt

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = [
            'front_left_steer_joint',
            'front_right_steer_joint',
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint',
        ]
        msg.position = [
            steering_left,
            steering_right,
            self.front_left_wheel_pos,
            self.front_right_wheel_pos,
            self.rear_left_wheel_pos,
            self.rear_right_wheel_pos,
        ]
        msg.velocity = [
            0.0,
            0.0,
            front_left_vel,
            front_right_vel,
            rear_left_vel,
            rear_right_vel,
        ]
        self.publisher.publish(msg)

    def _compute_steering(self, linear_v: float, yaw_rate: float) -> tuple[float, float]:
        if abs(linear_v) < 1e-6 or abs(yaw_rate) < 1e-6:
            return (0.0, 0.0)

        curvature = yaw_rate / linear_v
        if abs(curvature) < 1e-6:
            return (0.0, 0.0)

        turn_radius = 1.0 / curvature
        left_radius = turn_radius - (self.track_width / 2.0)
        right_radius = turn_radius + (self.track_width / 2.0)

        left_angle = math.atan(self.wheelbase / left_radius)
        right_angle = math.atan(self.wheelbase / right_radius)
        left_angle = max(-self.max_steering_angle, min(self.max_steering_angle, left_angle))
        right_angle = max(-self.max_steering_angle, min(self.max_steering_angle, right_angle))
        return (left_angle, right_angle)

    def _compute_wheel_angular_vel(self, linear_v: float, yaw_rate: float) -> tuple[float, float]:
        left_linear = linear_v - (yaw_rate * self.track_width / 2.0)
        right_linear = linear_v + (yaw_rate * self.track_width / 2.0)
        return (left_linear / self.wheel_radius, right_linear / self.wheel_radius)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelToJointStates()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()