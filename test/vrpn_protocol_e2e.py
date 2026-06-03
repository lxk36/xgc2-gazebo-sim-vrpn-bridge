#!/usr/bin/env python3
import unittest

import rospy
import rostest
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, PoseStamped, Twist


CLIENT_COUNT = 20


class GazeboSimVrpnProtocolE2ETest(unittest.TestCase):
    def setUp(self):
        self.received = {idx: [] for idx in range(1, CLIENT_COUNT + 1)}
        self.unrequested = {
            "client_01_uav02": [],
            "client_10_uav01": [],
            "client_20_uav19": [],
        }
        self.pub = rospy.Publisher("/test/gazebo/model_states", ModelStates, queue_size=10)
        self.subscribers = []

        for idx in range(1, CLIENT_COUNT + 1):
            client_ns = f"/client_{idx:02d}/vrpn_client_node"
            tracker = f"uav{idx}"
            self.subscribers.append(
                rospy.Subscriber(f"{client_ns}/{tracker}/pose", PoseStamped, self.received[idx].append)
            )

        self.subscribers.extend(
            [
                rospy.Subscriber(
                    "/client_01/vrpn_client_node/uav2/pose",
                    PoseStamped,
                    self.unrequested["client_01_uav02"].append,
                ),
                rospy.Subscriber(
                    "/client_10/vrpn_client_node/uav1/pose",
                    PoseStamped,
                    self.unrequested["client_10_uav01"].append,
                ),
                rospy.Subscriber(
                    "/client_20/vrpn_client_node/uav19/pose",
                    PoseStamped,
                    self.unrequested["client_20_uav19"].append,
                ),
            ]
        )

    def pose(self, x, y, z):
        msg = Pose()
        msg.position.x = x
        msg.position.y = y
        msg.position.z = z
        msg.orientation.w = 1.0
        return msg

    def model_states(self):
        msg = ModelStates()
        msg.name = [f"gazebo_uav_{idx:02d}" for idx in range(1, CLIENT_COUNT + 1)]
        msg.pose = [
            self.pose(float(idx), float(idx) * 0.1, float(idx) * 0.01)
            for idx in range(1, CLIENT_COUNT + 1)
        ]
        msg.twist = [Twist() for _ in range(CLIENT_COUNT)]
        return msg

    def publish_model_states(self, duration=0.5, hz=120.0):
        msg = self.model_states()
        rate = rospy.Rate(hz)
        end = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < end and not rospy.is_shutdown():
            self.pub.publish(msg)
            rate.sleep()

    def wait_for_all_clients(self, min_count, timeout=15.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        while rospy.Time.now() < deadline and not rospy.is_shutdown():
            if all(len(self.received[idx]) >= min_count for idx in range(1, CLIENT_COUNT + 1)):
                return True
            self.publish_model_states(duration=0.2, hz=120.0)
        return False

    def assert_latest_pose(self, idx, tol=2.0e-5):
        latest = self.received[idx][-1]
        self.assertAlmostEqual(latest.pose.position.x, float(idx), delta=tol)
        self.assertAlmostEqual(latest.pose.position.y, float(idx) * 0.1, delta=tol)
        self.assertAlmostEqual(latest.pose.position.z, float(idx) * 0.01, delta=tol)
        self.assertEqual(latest.header.frame_id, "world")

    def test_twenty_clients_receive_only_requested_trackers(self):
        rospy.sleep(1.0)
        self.publish_model_states(duration=3.0, hz=120.0)

        self.assertTrue(self.wait_for_all_clients(3))
        for idx in range(1, CLIENT_COUNT + 1):
            self.assert_latest_pose(idx)

        rospy.sleep(0.5)
        for key, messages in self.unrequested.items():
            self.assertEqual(len(messages), 0, f"{key} received an unrequested tracker")


if __name__ == "__main__":
    rospy.init_node("gazebo_sim_vrpn_protocol_e2e_test")
    rostest.rosrun("gazebo_sim_vrpn_bridge", "gazebo_sim_vrpn_protocol_e2e", GazeboSimVrpnProtocolE2ETest)
