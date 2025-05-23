# robot_web/ros_bridge_node.py
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ROSBridge(Node):
    def __init__(self):
        super().__init__('ros_bridge')

        # Get motor_names from ROS params (comma-separated string)
        self.declare_parameter('motor_names', 'motor1,motor2')
        motor_names_param = self.get_parameter('motor_names').get_parameter_value().string_value
        self.motor_names = motor_names_param.split(',')

        # Create publishers for each motor's cmd topic dynamically
        self.publishers_map = {}
        for motor_name in self.motor_names:
            topic_name = f'/{motor_name}/motor_cmd'
            self.publishers_map[motor_name] = self.create_publisher(String, topic_name, 10)
            self.get_logger().info(f"Created publisher for topic: {topic_name}")

    def publish_cmd(self, motor_name, cmd):
        pub = self.publishers_map.get(motor_name)
        if pub is None:
            self.get_logger().error(f"No publisher found for motor: {motor_name}")
            return False
        msg = String()
        msg.data = cmd
        self.get_logger().info(f"Publishing to {motor_name}: {cmd}")
        pub.publish(msg)
        return True

    # Example commands (can be extended)
    def set_velocity(self, motor_name, velocity):
        if self.publish_cmd(motor_name, f"set_vel {int(velocity)}"):
            self.publish_cmd(motor_name, "move_vel")
            return {"result": "OK", "velocity": velocity}
        return {"result": "Failed", "velocity": velocity}

    def jog(self, motor_name, direction):
        cmd = "jog_left" if direction == "left" else "jog_right"
        if self.publish_cmd(motor_name, cmd):
            return {"result": "OK", "direction": direction}
        return {"result": "Failed", "direction": direction}

    def stop_all(self):
        for motor_name in self.motor_names:
            self.publish_cmd(motor_name, "stop")
        return {"result": "Stopped"}

def main(args=None):
    rclpy.init(args=args)
    node = ROSBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
