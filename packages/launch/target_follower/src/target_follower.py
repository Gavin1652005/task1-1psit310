#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import AprilTagDetectionArray


class TargetFollower:

    def __init__(self):

        rospy.init_node('target_follower_node', anonymous=True)
        rospy.on_shutdown(self.clean_shutdown)

        robot_name = "mybota002445b"

        self.cmd_pub = rospy.Publisher(
            f'/{robot_name}/car_cmd_switch_node/cmd',
            Twist2DStamped,
            queue_size=1
        )

        rospy.Subscriber(
            f'/{robot_name}/apriltag_detector_node/detections',
            AprilTagDetectionArray,
            self.detection_cb,
            queue_size=1
        )

        # latest detection storage
        self.has_detection = False
        self.x = 0.0
        self.z = 0.0

        # control params
        self.goal_distance = 0.10

        self.kp_ang = 2.0
        self.kp_lin = 0.6

        # control loop at 10 Hz
        rospy.Timer(rospy.Duration(0.1), self.control_loop)

        rospy.loginfo("Target follower running (stable loop)")
        rospy.spin()

    # ---------------- STORE DETECTION ----------------
    def detection_cb(self, msg):

        if len(msg.detections) == 0:
            self.has_detection = False
            return

        tag = min(msg.detections, key=lambda d: d.transform.translation.z)

        self.x = tag.transform.translation.x
        self.z = tag.transform.translation.z

        self.has_detection = True

    # ---------------- MAIN CONTROL LOOP ----------------
    def control_loop(self, event):

        cmd = Twist2DStamped()

        # -------- SEARCH MODE --------
        if not self.has_detection:
            cmd.v = 0.0
            cmd.omega = 0.8   # smooth continuous spin
            self.cmd_pub.publish(cmd)
            return

        # -------- STOP MODE --------
        if self.z <= self.goal_distance:
            cmd.v = 0.0
            cmd.omega = 0.0
            self.cmd_pub.publish(cmd)
            return

        # -------- TRACK MODE --------
        omega = -self.kp_ang * self.x
        v = self.kp_lin * (self.z - self.goal_distance)

        # smoothing
        v *= max(0.2, 1.0 - abs(self.x) * 2.0)

        # clamp
        omega = max(-2.0, min(2.0, omega))
        v = max(0.0, min(0.25, v))

        cmd.v = v
        cmd.omega = omega

        self.cmd_pub.publish(cmd)

    # ---------------- SHUTDOWN ----------------
    def clean_shutdown(self):
        cmd = Twist2DStamped()
        cmd.v = 0.0
        cmd.omega = 0.0
        self.cmd_pub.publish(cmd)


if __name__ == '__main__':
    try:
        TargetFollower()
    except rospy.ROSInterruptException:
        pass
