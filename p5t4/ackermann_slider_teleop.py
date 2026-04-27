import threading
import tkinter as tk

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class AckermannSliderTeleop(Node):
    def __init__(self) -> None:
        super().__init__('ackermann_slider_teleop')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_yaw_rate', 1.5)

        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)

        self.publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.current_linear = 0.0
        self.current_yaw = 0.0

        period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.timer = self.create_timer(period, self.publish_twist)

        self.get_logger().info('ackermann_slider_teleop ready')
        self.get_logger().info(
            f'Publishing /cmd_vel at {self.publish_rate_hz:.1f} Hz '
            f'(max linear {self.max_linear_speed:.2f} m/s, max yaw {self.max_yaw_rate:.2f} rad/s)'
        )

    def set_linear(self, value: float) -> None:
        self.current_linear = value

    def set_yaw(self, value: float) -> None:
        self.current_yaw = value

    def stop(self) -> None:
        self.current_linear = 0.0
        self.current_yaw = 0.0

    def publish_twist(self) -> None:
        msg = Twist()
        msg.linear.x = self.current_linear
        msg.angular.z = self.current_yaw
        self.publisher.publish(msg)


def build_ui(node: AckermannSliderTeleop) -> tk.Tk:
    root = tk.Tk()
    root.title('Ackermann Slider Teleop')
    root.geometry('460x240')

    header = tk.Label(
        root,
        text='Drive command publisher (/cmd_vel)',
        font=('Arial', 12, 'bold'),
        pady=8,
    )
    header.pack()

    linear_label = tk.Label(root, text='Linear speed x [m/s]')
    linear_label.pack()

    linear_slider = tk.Scale(
        root,
        from_=-node.max_linear_speed,
        to=node.max_linear_speed,
        resolution=0.01,
        orient=tk.HORIZONTAL,
        length=380,
        command=lambda value: node.set_linear(float(value)),
    )
    linear_slider.set(0.0)
    linear_slider.pack()

    yaw_label = tk.Label(root, text='Yaw rate z [rad/s]')
    yaw_label.pack()

    yaw_slider = tk.Scale(
        root,
        from_=-node.max_yaw_rate,
        to=node.max_yaw_rate,
        resolution=0.01,
        orient=tk.HORIZONTAL,
        length=380,
        command=lambda value: node.set_yaw(float(value)),
    )
    yaw_slider.set(0.0)
    yaw_slider.pack()

    def stop_and_center() -> None:
        node.stop()
        linear_slider.set(0.0)
        yaw_slider.set(0.0)

    stop_button = tk.Button(root, text='STOP', command=stop_and_center, bg='#cc3333', fg='white')
    stop_button.pack(pady=10)

    return root


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AckermannSliderTeleop()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    root = build_ui(node)
    try:
        root.mainloop()
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()