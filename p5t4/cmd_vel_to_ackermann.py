import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class CmdVelToAckermann(Node):
    def __init__(self) -> None:
        super().__init__('cmd_vel_to_ackermann')

        self.declare_parameter('wheelbase', 0.215)
        self.declare_parameter('track_width', 0.16)
        self.declare_parameter('wheel_radius', 0.0325)
        self.declare_parameter('max_steering_angle', 0.52)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('steering_topic', '/steering_position_controller/commands')
        self.declare_parameter('traction_topic', '/traction_velocity_controller/commands')

        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.track_width = float(self.get_parameter('track_width').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.max_steering_angle = float(self.get_parameter('max_steering_angle').value)
        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        steering_topic = str(self.get_parameter('steering_topic').value)
        traction_topic = str(self.get_parameter('traction_topic').value)

        self.steering_pub = self.create_publisher(Float64MultiArray, steering_topic, 10)
        self.traction_pub = self.create_publisher(Float64MultiArray, traction_topic, 10)
        self.create_subscription(Twist, cmd_vel_topic, self.cmd_vel_callback, 10)

        self.get_logger().info('cmd_vel_to_ackermann ready')

    def cmd_vel_callback(self, msg: Twist) -> None:
        linear_v = float(msg.linear.x)
        yaw_rate = float(msg.angular.z)

        steering_left, steering_right = self._compute_steering(linear_v, yaw_rate)
        rear_left_vel, rear_right_vel = self._compute_rear_wheel_velocities(linear_v, yaw_rate)

        steering_msg = Float64MultiArray()
        steering_msg.data = [steering_left, steering_right]
        self.steering_pub.publish(steering_msg)

        traction_msg = Float64MultiArray()
        traction_msg.data = [rear_left_vel, rear_right_vel]
        self.traction_pub.publish(traction_msg)

    def _compute_steering(self, linear_v: float, yaw_rate: float) -> tuple[float, float]:
        if abs(linear_v) < 1e-6 or abs(yaw_rate) < 1e-6:
            return (0.0, 0.0)

        curvature = yaw_rate / linear_v
        center_angle = math.atan(self.wheelbase * curvature)
        center_angle = max(-self.max_steering_angle, min(self.max_steering_angle, center_angle))

        if abs(curvature) < 1e-6:
            return (center_angle, center_angle)

        turn_radius = 1.0 / curvature
        left_radius = turn_radius - (self.track_width / 2.0)
        right_radius = turn_radius + (self.track_width / 2.0)

        # For left turns (positive yaw), left wheel is the inner wheel.
        left_angle = math.atan(self.wheelbase / left_radius)
        right_angle = math.atan(self.wheelbase / right_radius)

        left_angle = max(-self.max_steering_angle, min(self.max_steering_angle, left_angle))
        right_angle = max(-self.max_steering_angle, min(self.max_steering_angle, right_angle))
        return (left_angle, right_angle)

    def _compute_rear_wheel_velocities(self, linear_v: float, yaw_rate: float) -> tuple[float, float]:
        left_linear = linear_v - (yaw_rate * self.track_width / 2.0)
        right_linear = linear_v + (yaw_rate * self.track_width / 2.0)
        return (left_linear / self.wheel_radius, right_linear / self.wheel_radius)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelToAckermann()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()