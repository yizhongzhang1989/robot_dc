import unittest
import rclpy
from robot_arm_web.web_server_node import RobotArmWebServer

class TestRobotArmWebServer(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = RobotArmWebServer()
    
    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()
    
    def test_node_initialization(self):
        self.assertIsNotNone(self.node)
        self.assertEqual(self.node.get_name(), 'robot_arm_web_server')
    
    def test_command_sending(self):
        result = self.node.send_command('power_on')
        self.assertEqual(result['status'], 'success')
        self.assertEqual(result['command'], 'power_on')

if __name__ == '__main__':
    unittest.main()
