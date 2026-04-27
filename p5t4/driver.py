import struct
#!/usr/bin/env python

import can
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, Int64


def clip(value: int, lower: int, upper: int) -> int:
    return max(lower, min(upper, value))


class DriverNode(Node):
    def __init__(self) -> None:
        super().__init__('driver')

        self.declare_parameter('can_channel', 'can0')
        self.declare_parameter('can_bitrate', 250000)
        self.declare_parameter('send_rate_hz', 20.0)
        self.declare_parameter('autonomous_enabled', True)
        self.declare_parameter('max_manual_steer_cmd', 100)
        self.declare_parameter('max_manual_throttle_forward', 23)
        self.declare_parameter('max_manual_throttle_reverse', 60)
        self.declare_parameter('wheel_radius', 0.0325)
        self.declare_parameter('autonomous_speed_scale', 15.0)
        self.declare_parameter('autonomous_output_scale', 5.0)
        self.declare_parameter('autonomous_throttle_sign', 1.0)
        self.declare_parameter('autonomous_steering_scale', 0.36)
        self.declare_parameter('autonomous_steering_sign', -1.0)

        can_channel = str(self.get_parameter('can_channel').value)
        can_bitrate = int(self.get_parameter('can_bitrate').value)
        send_rate_hz = float(self.get_parameter('send_rate_hz').value)
        self.autonomous_enabled = bool(self.get_parameter('autonomous_enabled').value)
        self.max_manual_steer_cmd = int(self.get_parameter('max_manual_steer_cmd').value)
        self.max_manual_throttle_forward = int(self.get_parameter('max_manual_throttle_forward').value)
        self.max_manual_throttle_reverse = int(self.get_parameter('max_manual_throttle_reverse').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.autonomous_speed_scale = float(self.get_parameter('autonomous_speed_scale').value)
        self.autonomous_output_scale = float(self.get_parameter('autonomous_output_scale').value)
        self.autonomous_throttle_sign = float(self.get_parameter('autonomous_throttle_sign').value)
        self.autonomous_steering_scale = float(self.get_parameter('autonomous_steering_scale').value)
        self.autonomous_steering_sign = float(self.get_parameter('autonomous_steering_sign').value)

        self.manual_throttle = 0
        self.manual_steer = 0
        self.autonomous_throttle = 0
        self.autonomous_steer = 0
        self._debug_count = 0

        self.bus = can.interface.Bus(
            bustype='socketcan',
            channel=can_channel,
            bitrate=can_bitrate,
        )

        self.create_subscription(Int64, '/manual_throttle', self.manual_throttle_callback, 1)
        self.create_subscription(Int64, '/manual_steering', self.manual_steering_callback, 1)
        self.create_subscription(
            Float64MultiArray,
            '/steering_position_controller/commands',
            self.autonomous_steering_callback,
            1,
        )
        self.create_subscription(
            Float64MultiArray,
            '/traction_velocity_controller/commands',
            self.autonomous_traction_callback,
            1,
        )
        self.create_subscription(Bool, '/autonomous_enabled', self.autonomous_enabled_callback, 1)

        period = 1.0 / max(send_rate_hz, 1.0)
        self.create_timer(period, self.send_can_command)

        mode = 'autonomous' if self.autonomous_enabled else 'manual'
        self.get_logger().info(f'driver ready on {can_channel} ({can_bitrate} bps), mode={mode}')

    def manual_throttle_callback(self, msg: Int64) -> None:
        self.manual_throttle = int(msg.data)

    def manual_steering_callback(self, msg: Int64) -> None:
        self.manual_steer = int(msg.data)

    def autonomous_enabled_callback(self, msg: Bool) -> None:
        self.autonomous_enabled = bool(msg.data)

    def autonomous_steering_callback(self, msg: Float64MultiArray) -> None:
        if not msg.data:
            return
        avg_steer_rad = float(sum(msg.data) / len(msg.data))
        steer_cmd = int(
            (avg_steer_rad / 0.52)
            * self.max_manual_steer_cmd
            * self.autonomous_steering_scale
            * self.autonomous_steering_sign
        )
        self.autonomous_steer = clip(steer_cmd, -self.max_manual_steer_cmd, self.max_manual_steer_cmd)

    def autonomous_traction_callback(self, msg: Float64MultiArray) -> None:
        if not msg.data:
            return
        avg_wheel_rad_s = float(sum(msg.data) / len(msg.data))
        linear_m_s = avg_wheel_rad_s * self.wheel_radius
        throttle_cmd = int(
            linear_m_s
            * self.autonomous_speed_scale
            * self.autonomous_output_scale
            * self.autonomous_throttle_sign
        )
        self.autonomous_throttle = clip(
            throttle_cmd,
            -self.max_manual_throttle_reverse,
            self.max_manual_throttle_forward,
        )

    def send_can_command(self) -> None:
        if self.autonomous_enabled:
            throttle = self.autonomous_throttle
            steer = self.autonomous_steer
            mode = 'AUTO'
        else:
            throttle = self.manual_throttle
            steer = self.manual_steer
            mode = 'MANUAL'

        # Final safety clamp before CAN packing/sending.
        throttle = clip(throttle, -self.max_manual_throttle_reverse, self.max_manual_throttle_forward)
        steer = clip(steer, -self.max_manual_steer_cmd, self.max_manual_steer_cmd)

        self._debug_count += 1
        if self._debug_count % 5 == 0:
            self.get_logger().info(
                'mode=%s manual(t=%d,s=%d) auto(t=%d,s=%d) can_out(t=%d,s=%d)'
                % (
                    mode,
                    self.manual_throttle,
                    self.manual_steer,
                    self.autonomous_throttle,
                    self.autonomous_steer,
                    throttle,
                    steer,
                )
            )

        try:
            can_data = struct.pack('>hhI', throttle, steer, 0)
            message = can.Message(arbitration_id=0x1, data=can_data, is_extended_id=False)
            self.bus.send(message)
        except Exception as error:
            self.get_logger().error(f'CAN send failed: {error}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DriverNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()