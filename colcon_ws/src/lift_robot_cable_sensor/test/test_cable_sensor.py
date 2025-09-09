#!/usr/bin/env python3

import unittest
import rclpy
from lift_robot_cable_sensor.cable_sensor_controller import CableSensorController

class TestCableSensor(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_controller_initialization(self):
        """测试控制器初始化"""
        # 这里可以添加实际的测试代码
        self.assertTrue(True)

if __name__ == '__main__':
    unittest.main()
