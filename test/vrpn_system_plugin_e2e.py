#!/usr/bin/env python3

import math
import time
import unittest

import rosgraph
import rosnode
import rospy
import rostest
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Pose, PoseStamped, TwistStamped
from std_srvs.srv import Empty


class GazeboVrpnSystemPluginE2ETest(unittest.TestCase):
    def setUp(self):
        self.pose_messages = {"uav1": [], "ugv9": [], "mecanum2": []}
        self.twist_messages = {"uav1": [], "ugv9": []}
        self.accel_messages = {"uav1": [], "ugv9": []}
        self.subscribers = []

        for tracker in self.pose_messages:
            self.subscribers.append(
                rospy.Subscriber(
                    f"/plugin_vrpn_client/{tracker}/pose",
                    PoseStamped,
                    lambda message, name=tracker: self.pose_messages[name].append(
                        (time.monotonic(), message)
                    ),
                )
            )
        for tracker in self.twist_messages:
            self.subscribers.append(
                rospy.Subscriber(
                    f"/plugin_vrpn_client/{tracker}/twist",
                    TwistStamped,
                    lambda message, name=tracker: self.twist_messages[name].append(
                        (time.monotonic(), message)
                    ),
                )
            )
            self.subscribers.append(
                rospy.Subscriber(
                    f"/plugin_vrpn_client/{tracker}/accel",
                    TwistStamped,
                    lambda message, name=tracker: self.accel_messages[name].append(
                        (time.monotonic(), message)
                    ),
                )
            )

        for service in (
            "/gazebo/spawn_sdf_model",
            "/gazebo/delete_model",
            "/gazebo/pause_physics",
            "/gazebo/unpause_physics",
            "/gazebo/reset_simulation",
        ):
            rospy.wait_for_service(service, timeout=20.0)

        self.spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        self.pause = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        self.unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        self.reset_simulation = rospy.ServiceProxy("/gazebo/reset_simulation", Empty)

    @staticmethod
    def wait_until(predicate, timeout_s=15.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if predicate():
                return True
            time.sleep(0.02)
        return False

    def wait_for_initial_streams(self):
        return self.wait_until(
            lambda: all(
                len(self.pose_messages[name]) >= 30 for name in ("uav1", "ugv9")
            )
            and all(
                len(self.twist_messages[name]) >= 10 for name in ("uav1", "ugv9")
            )
            and all(
                len(self.accel_messages[name]) >= 10 for name in ("uav1", "ugv9")
            ),
            timeout_s=20.0,
        )

    def assert_pose(self, tracker, expected, tolerance=2.0e-5):
        message = self.pose_messages[tracker][-1][1]
        self.assertEqual(message.header.frame_id, "world")
        self.assertAlmostEqual(message.pose.position.x, expected[0], delta=tolerance)
        self.assertAlmostEqual(message.pose.position.y, expected[1], delta=tolerance)
        self.assertAlmostEqual(message.pose.position.z, expected[2], delta=tolerance)
        self.assertAlmostEqual(message.pose.orientation.x, 0.0, delta=tolerance)
        self.assertAlmostEqual(message.pose.orientation.y, 0.0, delta=tolerance)
        self.assertAlmostEqual(message.pose.orientation.z, 0.0, delta=tolerance)
        self.assertAlmostEqual(abs(message.pose.orientation.w), 1.0, delta=tolerance)

    def assert_stationary_derivatives(self, tracker):
        twist = self.twist_messages[tracker][-1][1].twist
        accel = self.accel_messages[tracker][-1][1].twist
        values = (
            twist.linear.x,
            twist.linear.y,
            twist.linear.z,
            twist.angular.x,
            twist.angular.y,
            twist.angular.z,
            accel.linear.x,
            accel.linear.y,
            accel.linear.z,
            accel.angular.x,
            accel.angular.y,
            accel.angular.z,
        )
        self.assertTrue(all(math.isfinite(value) for value in values))
        self.assertLess(max(abs(value) for value in values), 1.0e-5)

    def assert_stream_quality_floor(self, tracker):
        receive_times = [record[0] for record in self.pose_messages[tracker][-80:]]
        self.assertGreaterEqual(len(receive_times), 30)
        duration_s = receive_times[-1] - receive_times[0]
        self.assertGreater(duration_s, 0.0)
        observed_rate_hz = (len(receive_times) - 1) / duration_s
        # This is deliberately a quality floor, not an equality check against
        # the legacy process. Lower jitter or a higher effective rate is valid.
        self.assertGreater(observed_rate_hz, 60.0)

        stamps = [
            record[1].header.stamp.to_sec()
            for record in self.pose_messages[tracker][-30:]
        ]
        self.assertTrue(
            all(right >= left for left, right in zip(stamps, stamps[1:]))
        )

    def assert_no_model_states_hot_path(self):
        _, subscribers, _ = rosgraph.Master(rospy.get_name()).getSystemState()
        model_state_subscribers = dict(subscribers).get("/gazebo/model_states", [])
        self.assertEqual(
            model_state_subscribers,
            [],
            f"in-process backend unexpectedly subscribed to /gazebo/model_states: "
            f"{model_state_subscribers}",
        )
        nodes = rosnode.get_node_names()
        self.assertIn("/gazebo", nodes)
        self.assertNotIn("/gazebo_vrpn_server", nodes)

    @staticmethod
    def dynamic_model_sdf():
        return """
        <sdf version="1.6">
          <model name="mecanum2">
            <static>true</static>
            <link name="base_link">
              <collision name="collision">
                <geometry><box><size>0.2 0.2 0.2</size></box></geometry>
              </collision>
            </link>
          </model>
        </sdf>
        """

    def spawn_dynamic_model(self, xyz):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = xyz
        pose.orientation.w = 1.0
        response = self.spawn_model(
            "mecanum2", self.dynamic_model_sdf(), "", pose, "world"
        )
        self.assertTrue(response.success, response.status_message)

    def test_contract_quality_and_dynamic_lifecycle(self):
        self.assertTrue(
            self.wait_for_initial_streams(), "initial tracker streams did not start"
        )

        # Auto mapping, manual mapping, body-to-tracker extrinsics, and all
        # three VRPN report types must remain available.
        self.assert_pose("uav1", (1.1, 1.8, 3.3))
        self.assert_pose("ugv9", (-1.0, 0.5, 0.2))
        for tracker in ("uav1", "ugv9"):
            self.assert_stationary_derivatives(tracker)
            self.assert_stream_quality_floor(tracker)
        self.assert_no_model_states_hot_path()

        # Pausing Gazebo must not tear down the VRPN server. Like the original
        # process, it repeats the last valid sample while simulation is paused.
        before_pause = len(self.pose_messages["uav1"])
        self.pause()
        time.sleep(0.35)
        after_pause = len(self.pose_messages["uav1"])
        self.assertGreater(after_pause - before_pause, 10)
        self.assert_pose("uav1", (1.1, 1.8, 3.3))
        self.unpause()

        # A simulation-time rollback resets derivative history without
        # interrupting tracker output.
        before_reset = len(self.pose_messages["uav1"])
        self.reset_simulation()
        self.assertTrue(
            self.wait_until(
                lambda: len(self.pose_messages["uav1"]) >= before_reset + 10
            ),
            "tracker stream did not recover after simulation reset",
        )
        self.assert_stationary_derivatives("uav1")

        # Dynamic insertion, deletion, and recreation exercise the same
        # low-rate discovery contract as the standalone server.
        self.spawn_dynamic_model((4.0, -2.0, 0.7))
        self.assertTrue(
            self.wait_until(lambda: len(self.pose_messages["mecanum2"]) >= 5),
            "dynamically inserted model was not exported",
        )
        self.assert_pose("mecanum2", (4.0, -2.0, 0.7))

        response = self.delete_model("mecanum2")
        self.assertTrue(response.success, response.status_message)
        time.sleep(0.25)
        count_after_drain = len(self.pose_messages["mecanum2"])
        time.sleep(0.35)
        self.assertLessEqual(
            len(self.pose_messages["mecanum2"]) - count_after_drain,
            2,
            "deleted model continued producing tracker samples",
        )

        self.spawn_dynamic_model((-3.0, 1.5, 1.2))
        recreated_start = len(self.pose_messages["mecanum2"])
        self.assertTrue(
            self.wait_until(
                lambda: len(self.pose_messages["mecanum2"]) >= recreated_start + 5
            ),
            "recreated model did not resume its existing tracker",
        )
        self.assert_pose("mecanum2", (-3.0, 1.5, 1.2))


if __name__ == "__main__":
    rospy.init_node("gazebo_sim_vrpn_system_plugin_e2e_test")
    rostest.rosrun(
        "gazebo_sim_vrpn_bridge",
        "gazebo_sim_vrpn_system_plugin_e2e",
        GazeboVrpnSystemPluginE2ETest,
    )
