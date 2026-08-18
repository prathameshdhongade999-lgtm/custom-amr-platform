#!/usr/bin/env python3
"""
eval_navigation_success.py

Runs N navigation trials to randomized goal poses and computes the
success-rate metric quoted in the README (92%). A "success" is: the
robot reaches the goal within tolerance without triggering a Nav2
recovery-behavior failure or timing out.

This is a standalone evaluation harness, not a ROS2 node — it drives
the stack via the NavigateToPose action client and logs pass/fail per
trial to a CSV for later analysis.
"""

import argparse
import csv
import random
import time
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

GOAL_TOLERANCE_M = 0.15
TRIAL_TIMEOUT_S = 60.0


class NavigationEvaluator(Node):
    def __init__(self, num_trials: int, bounds: tuple):
        super().__init__('navigation_evaluator')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.num_trials = num_trials
        self.bounds = bounds
        self.results = []

    def run(self):
        self._client.wait_for_server()
        for i in range(self.num_trials):
            x = random.uniform(*self.bounds[0])
            y = random.uniform(*self.bounds[1])
            success = self._run_trial(i, x, y)
            self.results.append((i, x, y, success))
            self.get_logger().info(f'Trial {i}: goal=({x:.2f},{y:.2f}) success={success}')

        rate = sum(1 for r in self.results if r[3]) / len(self.results)
        self.get_logger().info(f'Overall success rate: {rate * 100:.1f}%')
        self._write_csv()

    def _run_trial(self, trial_idx, x, y) -> bool:
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._build_pose(x, y)

        send_future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=TRIAL_TIMEOUT_S)
        result = result_future.result()
        return result is not None and result.status == 4  # STATUS_SUCCEEDED

    def _build_pose(self, x, y) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        return pose

    def _write_csv(self):
        with open('navigation_eval_results.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['trial', 'goal_x', 'goal_y', 'success'])
            writer.writerows(self.results)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--trials', type=int, default=50)
    parser.add_argument('--x-range', type=float, nargs=2, default=[-5.0, 5.0])
    parser.add_argument('--y-range', type=float, nargs=2, default=[-5.0, 5.0])
    args, _ = parser.parse_known_args(sys.argv[1:])

    rclpy.init()
    evaluator = NavigationEvaluator(args.trials, (args.x_range, args.y_range))
    evaluator.run()


if __name__ == '__main__':
    main()
