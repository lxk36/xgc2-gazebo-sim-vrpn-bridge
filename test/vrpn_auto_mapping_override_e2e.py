#!/usr/bin/env python3

import time
import unittest

import rospy
import rostest
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, PoseStamped, Twist


class GazeboSimVrpnAutoMappingOverrideE2ETest(unittest.TestCase):
    def setUp(self):
        self.received = {"uav1": [], "mecanum1": []}
        self.publisher = rospy.Publisher(
            "/test/auto_mapping/model_states", ModelStates, queue_size=10
        )
        self.subscribers = [
            rospy.Subscriber(
                "/auto_mapping_client/vrpn_client_node/{}/pose".format(model_name),
                PoseStamped,
                self.received[model_name].append,
            )
            for model_name in self.received
        ]

    @staticmethod
    def model_states():
        pose = Pose()
        pose.position.x = 1.25
        pose.position.y = -0.5
        pose.position.z = 0.75
        pose.orientation.w = 1.0

        message = ModelStates()
        mecanum_pose = Pose()
        mecanum_pose.position.x = -2.0
        mecanum_pose.position.y = 0.25
        mecanum_pose.position.z = 0.1
        mecanum_pose.orientation.w = 1.0

        message.name = ["uav1", "mecanum1"]
        message.pose = [pose, mecanum_pose]
        message.twist = [Twist(), Twist()]
        return message

    def test_explicit_private_parameter_overrides_yaml_auto_mapping(self):
        deadline = time.monotonic() + 15.0
        message = self.model_states()
        rate = rospy.Rate(120.0)
        while (
            time.monotonic() < deadline
            and not all(self.received.values())
            and not rospy.is_shutdown()
        ):
            self.publisher.publish(message)
            rate.sleep()

        self.assertTrue(self.received["uav1"], "explicit auto-track override did not export uav1")
        self.assertTrue(
            self.received["mecanum1"],
            "explicit auto-track override did not export canonical mecanum1",
        )

        uav = self.received["uav1"][-1]
        self.assertAlmostEqual(uav.pose.position.x, 1.25, delta=1.0e-4)
        self.assertAlmostEqual(uav.pose.position.y, -0.5, delta=1.0e-4)
        self.assertAlmostEqual(uav.pose.position.z, 0.75, delta=1.0e-4)
        self.assertEqual(uav.header.frame_id, "world")

        mecanum = self.received["mecanum1"][-1]
        self.assertAlmostEqual(mecanum.pose.position.x, -2.0, delta=1.0e-4)
        self.assertAlmostEqual(mecanum.pose.position.y, 0.25, delta=1.0e-4)
        self.assertAlmostEqual(mecanum.pose.position.z, 0.1, delta=1.0e-4)
        self.assertEqual(mecanum.header.frame_id, "world")


if __name__ == "__main__":
    rospy.init_node("gazebo_sim_vrpn_auto_mapping_override_e2e_test")
    rostest.rosrun(
        "gazebo_sim_vrpn_bridge",
        "gazebo_sim_vrpn_auto_mapping_override_e2e",
        GazeboSimVrpnAutoMappingOverrideE2ETest,
    )
