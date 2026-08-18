"""
visual_odometry_node.py

Estimates frame-to-frame camera motion from a monocular/stereo image
stream (feature tracking + essential-matrix/PnP pose recovery) and
publishes it as a continuous nav_msgs/Odometry stream. This is the
high-rate, drift-prone half of the AMCL + VO fusion described in the
project README — it fills in the motion estimate between AMCL's
lower-rate, globally-corrected pose updates.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge


class VisualOdometryNode(Node):
    def __init__(self):
        super().__init__('visual_odometry_node')
        self.bridge = CvBridge()
        self.orb = cv2.ORB_create(nfeatures=1000)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        self.prev_kp = None
        self.prev_des = None
        self.prev_gray = None

        # Accumulated pose (relative, will drift — corrected externally by AMCL via EKF)
        self.x, self.y, self.theta = 0.0, 0.0, 0.0

        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self._image_callback, 10
        )
        self.odom_pub = self.create_publisher(Odometry, '/visual_odom', 10)

        self.get_logger().info('Visual odometry node started.')

    def _image_callback(self, msg: Image):
        gray = cv2.cvtColor(
            self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'), cv2.COLOR_BGR2GRAY
        )
        kp, des = self.orb.detectAndCompute(gray, None)

        if self.prev_des is not None and des is not None:
            matches = self.matcher.match(self.prev_des, des)
            if len(matches) >= 8:
                self._estimate_motion(matches, kp)

        self.prev_kp, self.prev_des, self.prev_gray = kp, des, gray

    def _estimate_motion(self, matches, kp):
        pts_prev = np.float32([self.prev_kp[m.queryIdx].pt for m in matches])
        pts_curr = np.float32([kp[m.trainIdx].pt for m in matches])

        # NOTE: camera intrinsics (focal length, principal point) should come from
        # a calibration file in production use — placeholder values here.
        focal, pp = 700.0, (320.0, 240.0)
        E, mask = cv2.findEssentialMat(pts_curr, pts_prev, focal=focal, pp=pp,
                                        method=cv2.RANSAC, prob=0.999, threshold=1.0)
        if E is None:
            return
        _, R, t, _ = cv2.recoverPose(E, pts_curr, pts_prev, focal=focal, pp=pp)

        # Integrate relative motion into the accumulated (drifting) pose estimate
        dtheta = np.arctan2(R[1, 0], R[0, 0])
        self.theta += dtheta
        self.x += float(t[0]) * np.cos(self.theta)
        self.y += float(t[0]) * np.sin(self.theta)

        self._publish_odom()

    def _publish_odom(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = np.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = np.cos(self.theta / 2.0)
        # High covariance reflects that VO drifts over time and should be
        # trusted less than AMCL corrections in the fusion filter.
        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[35] = 0.1
        self.odom_pub.publish(odom)


def main():
    rclpy.init()
    node = VisualOdometryNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
