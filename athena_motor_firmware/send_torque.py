#!/usr/bin/env python3
import sys
import signal
import rclpy
from rclpy.node import Node
from athena_motor_interface.msg import TorqueCommand

RATE_HZ = 20
DECEL_NM_PER_S = 2.0


class TorqueSender(Node):
    def __init__(self, left: float, right: float):
        super().__init__("torque_sender")
        self.pub = self.create_publisher(
            TorqueCommand, "/athena/athena_motor_driver/forward_torque", 10
        )
        self.left = left
        self.right = right
        self.ramping_down = False
        self.timer = self.create_timer(1.0 / RATE_HZ, self.tick)

    def tick(self):
        if self.ramping_down:
            step = DECEL_NM_PER_S / RATE_HZ
            if self.left > 0:
                self.left = max(0.0, self.left - step)
            elif self.left < 0:
                self.left = min(0.0, self.left + step)
            if self.right > 0:
                self.right = max(0.0, self.right - step)
            elif self.right < 0:
                self.right = min(0.0, self.right + step)
            if self.left == 0.0 and self.right == 0.0:
                self.publish(0.0, 0.0)
                raise SystemExit

        self.publish(self.left, self.right)

    def publish(self, left: float, right: float):
        msg = TorqueCommand()
        msg.left = left
        msg.right = right
        self.pub.publish(msg)

    def start_ramp_down(self):
        self.ramping_down = True


def main():
    if len(sys.argv) not in (2, 3):
        print(f"Usage: {sys.argv[0]} <torque_nm> [right_torque_nm]")
        sys.exit(1)

    left = float(sys.argv[1])
    right = float(sys.argv[2]) if len(sys.argv) == 3 else left
    rclpy.init()
    node = TorqueSender(left, right)

    def on_sigint(sig, frame):
        node.get_logger().info("Ramping down...")
        node.start_ramp_down()

    signal.signal(signal.SIGINT, on_sigint)

    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
