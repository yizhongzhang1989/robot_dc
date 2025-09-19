import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image, Joy
from geometry_msgs.msg import PoseStamped
import time
from cv_bridge import CvBridge
import torchvision.transforms as transforms
from scipy.spatial.transform import Rotation as R
from rclpy.callback_groups import ReentrantCallbackGroup


def create_pose_msg(pose_matrix: np.ndarray = None, observe=False) -> PoseStamped:
    """
    Create a Pose message from either a 4x4 transformation matrix or pose dictionary
    
    Args:
        pose_matrix: 4x4 transformation matrix
        pose_dict: Dictionary with 'position' and 'orientation' keys
        
    Returns:
        Pose message
    """
    pose_msg = PoseStamped()
    
    # Extract position from transformation matrix
    pose_msg.pose.position.x = float(pose_matrix[0, 3])
    pose_msg.pose.position.y = float(pose_matrix[1, 3])
    pose_msg.pose.position.z = float(pose_matrix[2, 3])
    
    # Extract rotation matrix and convert to quaternion
    rotation_matrix = pose_matrix[:3, :3]
    r = R.from_matrix(rotation_matrix)
    quat = r.as_quat()  # Returns [x, y, z, w]
    
    pose_msg.pose.orientation.x = float(quat[0])
    pose_msg.pose.orientation.y = float(quat[1])
    pose_msg.pose.orientation.z = float(quat[2])
    pose_msg.pose.orientation.w = float(quat[3])
    # print("Manual orientation fixed")
    if observe:
        pose_msg.pose.orientation.x = 0.6925515674653875
        pose_msg.pose.orientation.y = 0.7153857677058679
        pose_msg.pose.orientation.z = -0.049108613211706946
        pose_msg.pose.orientation.w = 0.07863761106204165
    else:
        pose_msg.pose.orientation.x = 0.707
        pose_msg.pose.orientation.y = 0.707
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 0.0
    pose_msg.header.frame_id = "base_link"
    return pose_msg


class RobotStateProcessor(Node):
    def __init__(self):
        super().__init__('robot_state_node')
        self.bridge = CvBridge()
        self.rgb_image = None
        self.current_pose = None
        self.current_joy_msg = None
        self.callback_group = ReentrantCallbackGroup()
   
        # Transform for neural network input (same as data_loader.py)
        self.transform = transforms.Compose([
            transforms.Resize((448, 448)),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], 
                               std=[0.229, 0.224, 0.225])
        ])
        # Set up subscribers for RealSense camera topics
        rgb_sub = self.create_subscription(Image, '/robot_arm_camera/image_raw', self.rgb_callback, 10, callback_group=self.callback_group)
        pose_sub = self.create_subscription(PoseStamped, '/cartesian_motion_controller/current_pose',
                self.pose_callback, 10, callback_group=self.callback_group)
        
        # Add joystick subscription
        joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10, callback_group=self.callback_group)
        
        # New: target pose publisher and tolerances
        self.target_pose_pub = self.create_publisher(
            PoseStamped, 'target_frame', 10, callback_group=self.callback_group)
        
        self.pos_tol = 0.03  # meters
        self.ori_tol = 0.05  # radians
        self.get_logger().info("Target pose publisher initialized on /target_pose")

    def rgb_callback(self, msg):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"RGB conversion error: {e}")
    
    def pose_callback(self, msg):
        self.current_pose = msg

    def joy_callback(self, msg):
        self.current_joy_msg = msg

    def send_target_pose(self, target_pose: PoseStamped, pos_tol: float = None, ori_tol: float = None, publish_rate_hz: float = 10.0, absolute = True):
        pos_tol = self.pos_tol if pos_tol is None else pos_tol
        ori_tol = self.ori_tol if ori_tol is None else ori_tol
        dt = 1.0 / max(1.0, publish_rate_hz)
        cnt = 0
        while rclpy.ok():
            self.target_pose_pub.publish(target_pose)
            rclpy.spin_once(self, timeout_sec=0.0)
            if not absolute: return True
            if self.current_pose is not None:
                if self._pose_close(self.current_pose, target_pose, pos_tol, ori_tol):
                    return True
            time.sleep(dt)
            if cnt > 200:
                return True
            cnt += 1
            print("cnt:", cnt)
        return False

    @staticmethod
    def _pose_close(p1: PoseStamped, p2: PoseStamped, pos_tol: float, ori_tol: float) -> bool:
        dx = p1.pose.position.x - p2.pose.position.x
        dy = p1.pose.position.y - p2.pose.position.y
        dz = p1.pose.position.z - p2.pose.position.z
        print("p1:", p1.pose.position)
        print("p2:", p2.pose.position)
        pos_err = float(np.sqrt(dx*dx + dy*dy + dz*dz))
        q1 = np.array([p1.pose.orientation.w, p1.pose.orientation.x, p1.pose.orientation.y, p1.pose.orientation.z], dtype=np.float64)
        q2 = np.array([p2.pose.orientation.w, p2.pose.orientation.x, p2.pose.orientation.y, p2.pose.orientation.z], dtype=np.float64)
        q1 /= (np.linalg.norm(q1) + 1e-12)
        q2 /= (np.linalg.norm(q2) + 1e-12)
        dot = float(np.clip(np.dot(q1, q2), -1.0, 1.0))
        ang_err = 2.0 * float(np.arccos(abs(dot)))
        print("pos err:", pos_err)
        return pos_err <= pos_tol
