#!/usr/bin/env python3
import unittest

import rospy
import rostest
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, PoseStamped, Twist


class GazeboSimVrpnProtocolE2ETest(unittest.TestCase):
    def setUp(self):
        self.received = {
            "client_a_uav1": [],
            "client_a_uav2": [],
            "client_a_ugv1": [],
            "client_b_uav1": [],
            "client_b_ugv1": [],
        }
        self.pub = rospy.Publisher("/test/gazebo/model_states", ModelStates, queue_size=10)
        self.subscribers = [
            rospy.Subscriber("/client_a/vrpn_client_node/uav1/pose", PoseStamped, self.received["client_a_uav1"].append),
            rospy.Subscriber("/client_a/vrpn_client_node/uav2/pose", PoseStamped, self.received["client_a_uav2"].append),
            rospy.Subscriber("/client_a/vrpn_client_node/ugv1/pose", PoseStamped, self.received["client_a_ugv1"].append),
            rospy.Subscriber("/client_b/vrpn_client_node/uav1/pose", PoseStamped, self.received["client_b_uav1"].append),
            rospy.Subscriber("/client_b/vrpn_client_node/ugv1/pose", PoseStamped, self.received["client_b_ugv1"].append),
        ]

    def pose(self, x, y, z):
        msg = Pose()
        msg.position.x = x
        msg.position.y = y
        msg.position.z = z
        msg.orientation.w = 1.0
        return msg

    def publish_model_states(self, duration=3.0, hz=100.0):
        msg = ModelStates()
        msg.name = ["gazebo_uav_alpha", "gazebo_uav_beta", "gazebo_ugv_alpha"]
        msg.pose = [
            self.pose(1.0, 0.0, 0.0),
            self.pose(2.0, 0.0, 0.0),
            self.pose(0.0, 0.0, 3.0),
        ]
        msg.twist = [Twist(), Twist(), Twist()]

        rate = rospy.Rate(hz)
        end = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < end and not rospy.is_shutdown():
            self.pub.publish(msg)
            rate.sleep()

    def wait_for(self, key, min_count, timeout=8.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        while rospy.Time.now() < deadline and not rospy.is_shutdown():
            if len(self.received[key]) >= min_count:
                return True
            self.publish_model_states(duration=0.1, hz=100.0)
        return False

    def assert_latest_pose(self, key, x, y, z, tol=2.0e-5):
        latest = self.received[key][-1]
        self.assertAlmostEqual(latest.pose.position.x, x, delta=tol)
        self.assertAlmostEqual(latest.pose.position.y, y, delta=tol)
        self.assertAlmostEqual(latest.pose.position.z, z, delta=tol)
        self.assertEqual(latest.header.frame_id, "world")

    def test_requested_trackers_are_isolated_per_vrpn_client(self):
        rospy.sleep(0.5)
        self.publish_model_states(duration=2.0, hz=100.0)

        self.assertTrue(self.wait_for("client_a_uav1", 5))
        self.assertTrue(self.wait_for("client_b_ugv1", 5))
        self.assert_latest_pose("client_a_uav1", 1.0, 0.0, 0.0)
        self.assert_latest_pose("client_b_ugv1", 0.0, 0.0, 3.0)

        # Both clients connect to a server that has uav1/uav2/ugv1 registered,
        # but vrpn_client_ros should only publish explicitly requested trackers.
        rospy.sleep(0.5)
        self.assertEqual(len(self.received["client_a_uav2"]), 0)
        self.assertEqual(len(self.received["client_a_ugv1"]), 0)
        self.assertEqual(len(self.received["client_b_uav1"]), 0)


if __name__ == "__main__":
    rospy.init_node("gazebo_sim_vrpn_protocol_e2e_test")
    rostest.rosrun("gazebo_sim_vrpn_bridge", "gazebo_sim_vrpn_protocol_e2e", GazeboSimVrpnProtocolE2ETest)
