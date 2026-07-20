#!/usr/bin/env python3

import time
import unittest

import rospy
import rostest
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, PoseStamped, Twist


class GazeboSimVrpnAutoMappingOverrideE2ETest(unittest.TestCase):
    def setUp(self):
        self.received = []
        self.publisher = rospy.Publisher(
            "/test/auto_mapping/model_states", ModelStates, queue_size=10
        )
        self.subscriber = rospy.Subscriber(
            "/auto_mapping_client/vrpn_client_node/uav1/pose",
            PoseStamped,
            self.received.append,
        )

    @staticmethod
    def model_states():
        pose = Pose()
        pose.position.x = 1.25
        pose.position.y = -0.5
        pose.position.z = 0.75
        pose.orientation.w = 1.0

        message = ModelStates()
        message.name = ["uav1"]
        message.pose = [pose]
        message.twist = [Twist()]
        return message

    def test_explicit_private_parameter_overrides_yaml_auto_mapping(self):
        deadline = time.monotonic() + 15.0
        message = self.model_states()
        rate = rospy.Rate(120.0)
        while time.monotonic() < deadline and not self.received and not rospy.is_shutdown():
            self.publisher.publish(message)
            rate.sleep()

        self.assertTrue(self.received, "explicit auto-track override did not export uav1")
        latest = self.received[-1]
        self.assertAlmostEqual(latest.pose.position.x, 1.25, delta=1.0e-4)
        self.assertAlmostEqual(latest.pose.position.y, -0.5, delta=1.0e-4)
        self.assertAlmostEqual(latest.pose.position.z, 0.75, delta=1.0e-4)
        self.assertEqual(latest.header.frame_id, "world")


if __name__ == "__main__":
    rospy.init_node("gazebo_sim_vrpn_auto_mapping_override_e2e_test")
    rostest.rosrun(
        "gazebo_sim_vrpn_bridge",
        "gazebo_sim_vrpn_auto_mapping_override_e2e",
        GazeboSimVrpnAutoMappingOverrideE2ETest,
    )
