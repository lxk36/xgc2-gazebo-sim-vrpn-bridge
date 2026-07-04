#!/usr/bin/env python3
import bisect
import statistics
import time
import unittest

import rospy
import rostest
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, PoseStamped, Twist


class GazeboSimVrpnDelayE2ETest(unittest.TestCase):
    def setUp(self):
        self.fast_messages = []
        self.delayed_messages = []
        self.stream_start_s = time.monotonic()
        self.pub = rospy.Publisher("/test/delay/model_states", ModelStates, queue_size=20)
        self.fast_sub = rospy.Subscriber(
            "/delay_client/vrpn_client_node/uav1/pose",
            PoseStamped,
            lambda msg: self.fast_messages.append((time.monotonic(), msg)),
        )
        self.delayed_sub = rospy.Subscriber(
            "/delay_client/vrpn_client_node/uav2/pose",
            PoseStamped,
            lambda msg: self.delayed_messages.append((time.monotonic(), msg)),
        )

    def pose(self, x, y):
        msg = Pose()
        msg.position.x = x
        msg.position.y = y
        msg.orientation.w = 1.0
        return msg

    def model_states(self, x):
        msg = ModelStates()
        msg.name = ["source_fast", "source_delayed"]
        msg.pose = [self.pose(x, 0.0), self.pose(x, 1.0)]
        msg.twist = [Twist(), Twist()]
        return msg

    def publish_stream(self, duration_s=4.0, hz=240.0):
        rate = rospy.Rate(hz)
        end_s = time.monotonic() + duration_s
        while time.monotonic() < end_s and not rospy.is_shutdown():
            x = time.monotonic() - self.stream_start_s
            self.pub.publish(self.model_states(x))
            rate.sleep()

    def paired_deltas(self):
        fast = list(self.fast_messages)
        delayed = list(self.delayed_messages)
        fast_receive_times = [record[0] for record in fast]
        pose_deltas = []
        stamp_deltas = []
        cutoff_s = self.stream_start_s + 1.0

        for delayed_receive_s, delayed_msg in delayed:
            if delayed_receive_s < cutoff_s:
                continue

            index = bisect.bisect_left(fast_receive_times, delayed_receive_s)
            candidates = []
            if index < len(fast):
                candidates.append(fast[index])
            if index > 0:
                candidates.append(fast[index - 1])
            if not candidates:
                continue

            fast_receive_s, fast_msg = min(candidates, key=lambda record: abs(record[0] - delayed_receive_s))
            if abs(fast_receive_s - delayed_receive_s) > 0.05:
                continue

            pose_deltas.append(fast_msg.pose.position.x - delayed_msg.pose.position.x)
            stamp_deltas.append((fast_msg.header.stamp - delayed_msg.header.stamp).to_sec())

        return pose_deltas, stamp_deltas

    def assert_median_delay(self, values, label):
        self.assertGreaterEqual(len(values), 20, f"not enough paired {label} samples")
        median = statistics.median(values[-80:])
        self.assertGreater(median, 0.015, f"{label} median delay too small: {median:.6f}s")
        self.assertLess(median, 0.060, f"{label} median delay too large: {median:.6f}s")

    def test_tracker_fixed_delay_is_visible_in_pose_and_sample_timestamp(self):
        rospy.sleep(1.0)
        self.publish_stream(duration_s=4.0, hz=240.0)

        self.assertGreaterEqual(len(self.fast_messages), 20)
        self.assertGreaterEqual(len(self.delayed_messages), 20)

        pose_deltas, stamp_deltas = self.paired_deltas()
        self.assert_median_delay(pose_deltas, "pose")
        self.assert_median_delay(stamp_deltas, "sample timestamp")


if __name__ == "__main__":
    rospy.init_node("gazebo_sim_vrpn_delay_e2e_test")
    rostest.rosrun("gazebo_sim_vrpn_bridge", "gazebo_sim_vrpn_delay_e2e", GazeboSimVrpnDelayE2ETest)
