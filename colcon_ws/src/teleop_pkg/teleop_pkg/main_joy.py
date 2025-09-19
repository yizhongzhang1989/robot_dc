import rclpy
import cv2
import numpy as np
from .robot_state_processor import RobotStateProcessor, create_pose_msg

from copy import deepcopy
from .utils import DataSaver
from scipy.spatial.transform import Rotation as R


def main():
    rclpy.init()
    processor = RobotStateProcessor()
    HEIGHT_1 = 0.17  # button0 height
    HEIGHT_2 = 0.3  # button1 height (other specified height)
    
    # Movement gains
    XY_GAIN = 0.05

    # Define init pose
    qx, qy, qz, qw = 0.6925804329270558, 0.7154651897012169, -0.04887952889569517, 0.07779908680254266
    rotation = R.from_quat([qx, qy, qz, qw])
    rotation_matrix = rotation.as_matrix()
    pose_matrix_from_quat = np.eye(4)
    pose_matrix_from_quat[:3, :3] = rotation_matrix
    pose_matrix_from_quat[0, 3] = -0.05048721377287727  # x position
    pose_matrix_from_quat[1, 3] = -0.42946610406790175  # y position
    pose_matrix_from_quat[2, 3] = 0.7610675973184527  # z position
    init_pose = create_pose_msg(pose_matrix=pose_matrix_from_quat, observe=True)
    
    # State variables
    origin_pose = None
    action_pose = None
    prev_buttons = [0] * 10  # Previous button states for edge detection

    # Helper function to create pose matrix from position and orientation
    def create_pose_matrix_from_position_orientation(position, orientation_quat):
        pose_matrix = np.eye(4)
        pose_matrix[0, 3] = position[0]
        pose_matrix[1, 3] = position[1]  
        pose_matrix[2, 3] = position[2]
        rotation = R.from_quat([orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w])
        pose_matrix[:3, :3] = rotation.as_matrix()
        return pose_matrix

    # Helper function to send pose with specific position while keeping current orientation
    def send_pose_to_position(position, orientation_quat, absolute):
        pose_matrix = create_pose_matrix_from_position_orientation(position, orientation_quat)
        target_pose_msg = create_pose_msg(pose_matrix=pose_matrix)
        processor.send_target_pose(target_pose_msg, absolute=absolute)

    # Helper function to handle height change buttons
    def handle_height_button(button_num, target_height, cur_pos, cur_orientation):
        new_pos = cur_pos.copy()
        new_pos[2] = target_height
        #print(f"Button {button_num} pressed: Moving to height {target_height}")
        send_pose_to_position(new_pos, cur_orientation, absolute=True)


    saver = DataSaver()
    
    processor.send_target_pose(init_pose)  # send init pose once
    init_img = None
    while rclpy.ok():
        rclpy.spin_once(processor, timeout_sec=0.1)
        
        if processor.rgb_image is not None:
            rgb_image = cv2.cvtColor(processor.rgb_image.copy(), code=cv2.COLOR_RGB2BGR)
            #print(rgb_image.shape)
            if init_img is None:
                init_img = rgb_image.copy()
        if processor.current_pose is not None and processor.current_joy_msg is not None:
            cur_pose = processor.current_pose
            
            # Get current position
            cur_pos = np.array([
                cur_pose.pose.position.x,
                cur_pose.pose.position.y,
                cur_pose.pose.position.z
            ])
            
            # Get current orientation (keep the same)
            cur_orientation = cur_pose.pose.orientation
            
            # Calculate movement from joystick axes
            x_movement = processor.current_joy_msg.axes[4] * XY_GAIN  # left stick horizontal
            y_movement = - processor.current_joy_msg.axes[5] * XY_GAIN  # left stick vertical
            
            # Apply continuous movement based on joystick input
            new_pos = cur_pos.copy()
            new_pos[0] += x_movement
            new_pos[1] += y_movement
            
            # Handle button presses (edge detection)
            buttons = processor.current_joy_msg.buttons
            
            send_pose_to_position(new_pos, cur_orientation, absolute=False)
  
            # Button 6: Move to HEIGHT_1
            if buttons[6] and not prev_buttons[6]:
                handle_height_button(0, HEIGHT_1, cur_pos, cur_orientation)
            
            # Button 4: Move to HEIGHT_2 (other specified height)
            elif buttons[4] and not prev_buttons[4]:
                handle_height_button(1, HEIGHT_2, cur_pos, cur_orientation)
            
            # Button 5: Capture current pose as origin_pose
            elif buttons[5] and not prev_buttons[5]:
                origin_pose = deepcopy(cur_pose)
            
            # Button 7: Capture as action_pose, save, move to HEIGHT_2, then init
            elif buttons[7] and not prev_buttons[7]:
                if origin_pose is not None:
                    action_pose = deepcopy(cur_pose)
                    
                    # Save the data
                    if init_img is not None:
                        #rgb_image = cv2.cvtColor(processor.rgb_image.copy(), code=cv2.COLOR_RGB2BGR)
                        saver.save(
                            cv2.cvtColor(init_img, cv2.COLOR_RGB2BGR),
                            origin_pose,
                            action_pose,
                        )
                        init_img = None
                    # Move to HEIGHT_2, then to init pose
                    current_pos = np.array([cur_pose.pose.position.x, cur_pose.pose.position.y, HEIGHT_2])
                    send_pose_to_position(current_pos, cur_pose.pose.orientation, absolute=True)
                    processor.send_target_pose(init_pose, absolute=True)                    
                    # Reset captured poses
                    origin_pose = None
                    action_pose = None

                else:
                    print("Button 3 pressed: No origin pose captured yet!")
            elif buttons[0] and not prev_buttons[0]:
                init_img = None
                # Move to HEIGHT_2, then to init pose
                current_pos = np.array([cur_pose.pose.position.x, cur_pose.pose.position.y, HEIGHT_2])
                send_pose_to_position(current_pos, cur_pose.pose.orientation, absolute=True)
                processor.send_target_pose(init_pose, absolute=True)
                
                # Reset captured poses
                origin_pose = None
                action_pose = None

            # Update previous button states
            prev_buttons = deepcopy(buttons)

    cv2.destroyAllWindows()
    processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()