import pytest
import unittest
from duco_robot_arm_state.robot_data import RobotData
import struct


class TestRobotData(unittest.TestCase):
    def test_robot_data_parsing(self):
        """Test that RobotData can parse valid byte data."""
        # Create dummy data (608 bytes minimum)
        dummy_data = bytearray(1468)  # Full frame size
        
        # Fill with some test values
        # Joint positions (first 28 bytes, 7 floats)
        for i in range(7):
            struct.pack_into('f', dummy_data, i*4, float(i))
        
        # TCP position (bytes 368-391, 6 floats)
        for i in range(6):
            struct.pack_into('f', dummy_data, 368 + i*4, float(i + 0.5))
        
        # Parse the data
        robot_data = RobotData.from_bytes(dummy_data)
        
        # Check that the data was parsed correctly
        self.assertEqual(len(robot_data.jointActualPosition), 7)
        self.assertEqual(len(robot_data.TCPActualPosition), 6)
        self.assertEqual(robot_data.jointActualPosition[0], 0.0)
        self.assertEqual(robot_data.TCPActualPosition[0], 0.5)

    def test_insufficient_data(self):
        """Test that RobotData raises error for insufficient data."""
        with self.assertRaises(ValueError):
            RobotData.from_bytes(b'too short')

    def test_get_tcp_pose(self):
        """Test the get_tcp_pose method."""
        dummy_data = bytearray(1468)
        
        # Set TCP position data
        tcp_values = [1.0, 2.0, 3.0, 0.1, 0.2, 0.3]
        for i, value in enumerate(tcp_values):
            struct.pack_into('f', dummy_data, 368 + i*4, value)
        
        robot_data = RobotData.from_bytes(dummy_data)
        tcp_pose = robot_data.get_tcp_pose()
        
        self.assertEqual(len(tcp_pose), 6)
        self.assertEqual(tcp_pose, tcp_values)


if __name__ == '__main__':
    unittest.main()
