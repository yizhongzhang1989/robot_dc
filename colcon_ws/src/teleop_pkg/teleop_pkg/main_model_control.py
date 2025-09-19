import torch
import rclpy
import cv2
import time
import numpy as np
from copy import deepcopy
import torchvision.transforms as T
from PIL import Image
from scipy.spatial.transform import Rotation as R

from .robot_state_processor import RobotStateProcessor, create_pose_msg

from .model import Model


def reconstruct_pose(pixel, known_z, K, cam2end, robot_pose, img_size=(1920, 1080)):
    x_pixel_orig = img_size[0] - pixel[0]
    y_pixel_orig = img_size[1] - pixel[1]
    
    K_inv = np.linalg.inv(K)
    pixel_homo = np.array([x_pixel_orig, y_pixel_orig, 1.0])
    camera_direction = K_inv @ pixel_homo
    
    # We need to find the scaling factor such that the resulting world Z equals known_z
    # Let's work backwards: we know the final world Z should be known_z
    
    # Transform matrices
    robot_pose = np.asarray(robot_pose, dtype=np.float64)
    cam2end_inv = np.linalg.inv(cam2end)
    
    # For a point at depth 'd' in camera frame:
    # pos_camera = camera_direction * d
    # pos_world = robot_pose @ cam2end_inv @ [pos_camera; 1]
    # We want pos_world[2] = known_z
    
    # Let's solve for the depth 'd' that gives us the desired world Z
    # This requires solving: (robot_pose @ cam2end_inv @ [camera_direction * d; 1])[2] = known_z
    
    # Build transformation from camera to world
    T_camera_to_world = robot_pose @ cam2end_inv
    
    # Extract relevant components for Z calculation
    # pos_world = T_camera_to_world @ [camera_direction * d; 1]
    # pos_world[2] = T_camera_to_world[2, :3] @ (camera_direction * d) + T_camera_to_world[2, 3]
    
    rotation_z = T_camera_to_world[2, :3]  # Third row, first 3 columns
    translation_z = T_camera_to_world[2, 3]  # Third row, fourth column
    
    # Solve for depth: known_z = rotation_z @ (camera_direction * d) + translation_z
    # known_z = d * (rotation_z @ camera_direction) + translation_z
    # d = (known_z - translation_z) / (rotation_z @ camera_direction)
    
    denominator = np.dot(rotation_z, camera_direction)
    if abs(denominator) < 1e-8:
        # Ray is parallel to the Z plane, use a fallback method
        # Use middle depth as approximation
        depth = 1.0  # 1 meter as default
    else:
        depth = (known_z - translation_z) / denominator
    
    # Now compute 3D camera coordinates
    pos_camera = camera_direction * depth
    
    # Convert from camera frame to local frame
    ee_pos_camera = np.eye(4)
    ee_pos_camera[:3, 3] = pos_camera
    ee_pos_local = cam2end_inv @ ee_pos_camera
    
    # Convert from local frame to world frame
    ee_pos_world_mat = robot_pose @ ee_pos_local
    ee_pos_world = ee_pos_world_mat[:3, 3]
    
    return ee_pos_world

def grid_idx_to_pixel(grid_idx, target_size=(1920, 1080), grid_size=32):
    """Convert grid index back to pixel coordinates"""
    # Convert linear index to (x, y): index -> (index % width, index // width)
    grid_x = grid_idx % grid_size
    grid_y = grid_idx // grid_size
    
    # Scale from 32x32 to 448x448 first
    pixel_x_448 = (grid_x + 0.5) * 448 / grid_size
    pixel_y_448 = (grid_y + 0.5) * 448 / grid_size
    
    # Scale from 448x448 to target size
    target_w, target_h = target_size
    pixel_x = pixel_x_448 * target_w / 448
    pixel_y = pixel_y_448 * target_h / 448
    
    return (int(pixel_x), int(pixel_y))

def main():
    k = np.array([
        [1583.713421584196, 0.0, 982.8706765419257],
        [0.0, 1583.7034747152047, 543.7404915500364],
        [0.0, 0.0, 1.0
    ]])

    # Create push_pose matrix from position and orientation
    push_position = [0.36540161569463653, -0.6793462106842818, 0.2287114199817345]
    push_orientation = [0., 1.0, 0.0, 0.0]
    
    # fixed z height for reconstruction (must be changed accordingly)
    targ_z = 0.2

    # Convert quaternion to rotation matrix
    push_rotation = R.from_quat(push_orientation).as_matrix()

    # Create 4x4 pose matrix
    push_pose_matrix = np.eye(4)
    push_pose_matrix[:3, :3] = push_rotation
    push_pose_matrix[:3, 3] = push_position
    push_pose_up = deepcopy(push_pose_matrix)
    push_pose_up[2, 3] += 0.05  # Move 10cm higher
    push_pose = create_pose_msg(pose_matrix=push_pose_matrix, observe=True)
    push_pose_up = create_pose_msg(pose_matrix=push_pose_up, observe=True)
    # Add robot_pose matrix
    quat = [0.6925515674653875, 0.7153857677058679, -0.049108613211706946, 0.07863761106204165]
    position = [-0.05048721377287727, -0.42946610406790175, 0.7610675973184527]
    rotation_matrix = R.from_quat(quat).as_matrix()
    robot_pose = np.eye(4)
    robot_pose[:3, :3] = rotation_matrix
    robot_pose[:3, 3] = position
    
    cam2end = np.array([
        [
        0.017523325506620804,
        0.9761094151864712,
        0.21657179559562753,
        -0.13159984316790044
        ],
        [
        -0.9998372665086457,
        0.016178545501719068,
        0.007980925132363493,
        -0.003162021093926427
        ],
        [
        0.004286439514165484,
        -0.21667640446013972,
        0.9762340714124444,
        0.09821257411689592
        ],
        [
        0.0,
        0.0,
        0.0,
        1.0
        ]
    ])



    qx, qy, qz, qw = 0.6925804329270558, 0.7154651897012169, -0.04887952889569517, 0.07779908680254266

    # Create rotation matrix from quaternion
    rotation = R.from_quat([qx, qy, qz, qw])
    rotation_matrix = rotation.as_matrix()

    # Create 4x4 pose matrix with the given position and converted rotation
    pose_matrix_from_quat = np.eye(4)
    pose_matrix_from_quat[:3, :3] = rotation_matrix
    pose_matrix_from_quat[0, 3] = -0.05048721377287727  # x position
    pose_matrix_from_quat[1, 3] = -0.42946610406790175  # y position
    pose_matrix_from_quat[2, 3] = 0.7610675973184527  # z position
    init_pose = create_pose_msg(pose_matrix=pose_matrix_from_quat, observe=True)
    pose_matrix_from_quat[2, 3] = 0.3610675973184527  # z position lower
    push_ready_pose = create_pose_msg(pose_matrix=pose_matrix_from_quat, observe=True)
    # Set frame_id for init_pose
    print("init pose:", init_pose)


    rclpy.init()
    processor = RobotStateProcessor()
    processor.send_target_pose(init_pose) # send init pose once
    target_size = (448, 448)
    transform = T.Compose([T.Resize(target_size), T.ToTensor(), T.Normalize([0.485,0.456,0.406],[0.229,0.224,0.225])])

    checkpoint_path = "/home/a/ws_heecheol/server_pushing/model_pkg/result/model_epoch_19.pth"  # Replace XX with the desired epoch number
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    # Initialize and load model
    model = Model().to(device)
    model.load_state_dict(torch.load(checkpoint_path, map_location=device))
    model.eval()

    while rclpy.ok():
        y = input("Go? (Press any key and enter)")
        rclpy.spin_once(processor, timeout_sec=0.1)

        if y == 'G':
            processor.send_target_pose(push_ready_pose, pos_tol=0.01, absolute=True)
            processor.send_target_pose(push_pose_up, pos_tol=0.01, absolute=True)
            processor.send_target_pose(push_pose, pos_tol=0.01, absolute=True)
        else:
            if processor.rgb_image is not None and processor.current_pose is not None:
                rgb_image = cv2.cvtColor(processor.rgb_image.copy(), code=cv2.COLOR_RGB2BGR)
                
                #rgb_image = processor.rgb_image.copy()
                pil_image = Image.fromarray(rgb_image)
                with torch.no_grad():
                    img = transform(pil_image).unsqueeze(0).to(device)
                    outputs = model.forward_test(img)
                    origin_predictions = outputs['origin_predictions'][0].to('cpu').detach().numpy()
                    action_predictions = outputs['action_predictions'][0].to('cpu').detach().numpy()
                origin_pixel = grid_idx_to_pixel(origin_predictions, target_size=(1920,1080))
                action_pixel = grid_idx_to_pixel(action_predictions, target_size=(1920,1080))
                cv2.circle(rgb_image, origin_pixel, 20, (255, 255, 255), 10)
                cv2.circle(rgb_image, action_pixel, 20, (255, 0, 0), 10)
                cv2.imwrite("img.png", cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB))
                
                # Convert pixel coordinates to world positions
                origin_pos = reconstruct_pose(origin_pixel, targ_z, k, cam2end, robot_pose)
                action_pos = reconstruct_pose(action_pixel, targ_z, k, cam2end, robot_pose)
                # Create pose matrices from positions
                origin = np.eye(4)
                origin[:3, 3] = origin_pos
                origin_top = deepcopy(origin)
                origin_top[2, 3] += 0.1  # Move 10cm higher
                origin_top_pose = create_pose_msg(pose_matrix=origin_top)
                processor.send_target_pose(origin_top_pose, pos_tol=0.01, absolute=True)
                origin_pose = create_pose_msg(pose_matrix=origin)
                processor.send_target_pose(origin_pose, pos_tol=0.01, absolute=True)
                action_pos[0] -= 0.05 # additional bonus to make sure push
                action = np.eye(4)
                action[:3, 3] = action_pos
                action_pose = create_pose_msg(pose_matrix=action)
                processor.send_target_pose(action_pose, pos_tol=0.01, absolute=True)
                # Back to origin pose for safety
                processor.send_target_pose(origin_pose, pos_tol=0.01, absolute=True)

                processor.send_target_pose(init_pose, pos_tol=0.01, absolute=True) # send init pose once
                
    cv2.destroyAllWindows()
    processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()