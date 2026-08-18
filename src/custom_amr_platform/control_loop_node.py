"""
control_loop_node.py

The 15Hz control loop referenced in the README's headline result.
Consumes the FUSED odometry (robot_localization EKF output — AMCL +
visual odometry combined, not raw AMCL) and the current Nav2 velocity
command, and republishes it through the ros2_control hardware
interface at a fixed, stable rate.

Why a fixed-rate wrapper at all, instead of just letting Nav2 drive
the hardware interface directly? It decouples the control loop's
timing from whatever rate upstream planners/localization happen to
publish at, so the wheel commands stay smooth even if e.g. Nav2's
controller_server briefly hiccups.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

CONTROL_RATE_HZ = 15.0


class ControlLoopNode(Node):
    def __init__(self):
        super().__init__('control_loop_node')

        self._latest_cmd = Twist()
        self._latest_fused_odom = None

        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel_nav', self._cmd_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self._odom_callback, 10
        )  # /odometry/filtered = robot_localization EKF fusion output
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(1.0 / CONTROL_RATE_HZ, self._tick)
        self.get_logger().info(f'Control loop running at {CONTROL_RATE_HZ} Hz.')

    def _cmd_callback(self, msg: Twist):
        self._latest_cmd = msg

    def _odom_callback(self, msg: Odometry):
        self._latest_fused_odom = msg

    def _tick(self):
        if self._latest_fused_odom is None:
            self.get_logger().warn_once('Waiting for fused odometry before publishing commands.')
            return
        # In production this is where a safety/limits check against the fused
        # pose estimate would live before commands reach the hardware interface.
        self.cmd_pub.publish(self._latest_cmd)


def main():
    rclpy.init()
    node = ControlLoopNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
