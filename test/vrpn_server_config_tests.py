#!/usr/bin/env python3

import os
import subprocess
import unittest


PACKAGE = "gazebo_sim_vrpn_bridge"


class VrpnServerConfigTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.package_dir = subprocess.check_output(["rospack", "find", PACKAGE], text=True).strip()

    def dump_params(self, *args):
        command = ["roslaunch", "--dump-params", PACKAGE, "vrpn_server.launch"]
        command.extend(args)
        return subprocess.check_output(command, text=True)

    def config_path(self, filename):
        return os.path.join(self.package_dir, "config", filename)

    def test_default_config_loads_no_delay_namespace(self):
        params = self.dump_params()

        self.assertNotIn("/gazebo_vrpn_server/delay/", params)
        self.assertNotIn("/gazebo_vrpn_server/delay_enabled", params)
        self.assertNotIn("/gazebo_vrpn_server/delay_timestamp_policy", params)
        self.assertNotIn("/gazebo_vrpn_server/delay_seed", params)

    def test_simple_delay_config_is_explicit_and_clamped_to_30_ms(self):
        params = self.dump_params("config:=" + self.config_path("vrpn_server_delay_simple.yaml"))

        self.assertIn("/gazebo_vrpn_server/delay/enabled: true", params)
        self.assertIn("/gazebo_vrpn_server/delay/timestamp_policy: send_time", params)
        self.assertIn("/gazebo_vrpn_server/delay/max_delay_ms: 30.0", params)
        self.assertIn("/gazebo_vrpn_server/delay/common/base_ms: 15.0", params)

    def test_complex_delay_config_is_explicit_and_clamped_to_30_ms(self):
        params = self.dump_params("config:=" + self.config_path("vrpn_server_delay_complex.yaml"))

        self.assertIn("/gazebo_vrpn_server/delay/enabled: true", params)
        self.assertIn("/gazebo_vrpn_server/delay/timestamp_policy: sample_time", params)
        self.assertIn("/gazebo_vrpn_server/delay/max_delay_ms: 30.0", params)
        self.assertIn("/gazebo_vrpn_server/delay/trackers/uav1/max_delay_ms: 30.0", params)
        self.assertIn("/gazebo_vrpn_server/delay/trackers/ugv1/max_delay_ms: 30.0", params)

    def test_launch_overrides_are_absent_until_passed(self):
        params = self.dump_params(
            "delay_enabled:=true",
            "delay_timestamp_policy:=sample_time",
            "delay_seed:=9",
        )

        self.assertIn("/gazebo_vrpn_server/delay_enabled: true", params)
        self.assertIn("/gazebo_vrpn_server/delay_timestamp_policy: sample_time", params)
        self.assertIn("/gazebo_vrpn_server/delay_seed: 9", params)


if __name__ == "__main__":
    unittest.main()
