import numpy as np


def project_pose(ee_pos_world, K, cam2end, robot_pose, img_size=(1920, 1080)):
    ee_pos_mat = np.eye(4)
    ee_pos_mat[:3, 3] = ee_pos_world
    robot_pose = np.asarray(robot_pose, dtype=np.float64)
    ee_pos_local = np.linalg.pinv(robot_pose) @ ee_pos_mat
    ee_pos_camera = cam2end @ ee_pos_local
    pos_camera = ee_pos_camera[:3, 3]  # Extract [x, y, z] in camera frame
    pos_image_homo = K @ pos_camera  # 3x3 @ 3x1 = 3x1
    x_pixel = pos_image_homo[0] / pos_image_homo[2]
    y_pixel = pos_image_homo[1] / pos_image_homo[2]
    x_pixel = int(round(x_pixel))
    y_pixel = int(round(y_pixel))
    return img_size[0] - x_pixel, img_size[1] - y_pixel
