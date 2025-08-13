import os
import sys
import numpy as np
from typing import Sequence, Tuple, List
from numpy.typing import ArrayLike
import json
from glob import glob
import cv2
import nlopt
import argparse

# #=============================General functions of calibration====================================
# Generate 3D coordinates of chessboard corners for all images
def get_objpoints(num_images: int, XX: int, YY: int, L: float) -> Sequence[ArrayLike]:
    
    objpoints = []  # shape = num*(88,3)
    for i in range(0, num_images):

        objp = np.zeros((XX * YY, 3), np.float32)
        objp[:, :2] = np.mgrid[0:XX, 0:YY].T.reshape(-1, 2)
        objp *= L
        objpoints.append(objp)
    
    return objpoints

# Calculate the average reprojection error for all images
def calculate_reproject_error_fast(imgpoints, objpoints, 
                                   rvecs_target2cam, tvecs_target2cam, mtx, dist, 
                                   image_paths=None, XX=None, YY=None, vis=False, save_fig=False, save_dir='../colcon_ws/src/camera_calibrate/output',
                                   verbose=True):

    num_images = len(imgpoints)
    
    mean_error = 0

    # Calculate reprojection error for each image
    for image_idx in range(0, num_images):

        imgpoints2, _ = cv2.projectPoints(objpoints[image_idx], rvecs_target2cam[image_idx], tvecs_target2cam[image_idx], mtx, dist)
        error = cv2.norm(imgpoints[image_idx], imgpoints2, cv2.NORM_L2) / len(imgpoints2)

        if verbose:
            print(f"{image_idx} error: {error}")

        mean_error += error

        # vis or save_fig decide whether to display or save the reprojected corners image
        if vis or save_fig:
            imagei = cv2.imread(image_paths[image_idx])            
            img_draw = cv2.drawChessboardCorners(imagei, (XX, YY), imgpoints2, True)

            if save_fig:
                if not os.path.isdir(save_dir):
                    os.makedirs(save_dir)
                cv2.imwrite(f'{save_dir}/{image_idx}_reproject_corners.png', img_draw)

            if vis:
                cv2.imshow('img_draw', img_draw)
                cv2.waitKey(0)

    # verbose - decide whether to print detailed debugging and progress information
    if verbose:
        print( "total error: {}".format(mean_error/len(objpoints)) )

    return mean_error/len(objpoints)

# calculate the reprojection error of single image
def calculate_single_image_reprojection_error(image_path, rvec_target2cam, tvec_target2cam, mtx, dist, XX, YY, L, vis=False, save_fig=False, save_dir='../colcon_ws/src/camera_calibrate/output', verbose=True):
    
    imgpoints = get_chessboard_corners([image_path], XX, YY)
    
    objpoints = get_objpoints(1, XX, YY, L)

    # use objpoints[0]和imgpoints[0] because we are dealing with a single image
    reprojected_imgpoints, _ = cv2.projectPoints(objpoints[0], rvec_target2cam, tvec_target2cam, mtx, dist)
    
    error = cv2.norm(imgpoints[0], reprojected_imgpoints, cv2.NORM_L2)/len(reprojected_imgpoints)

    # ======================use to debug==================================
    if verbose:
        print(f"{image_path} reprojection error: {error}")

    # if vis or save_fig:
    #     img = cv2.imread(image_path)
    #     img_draw = cv2.drawChessboardCorners(img, (XX, YY), reprojected_imgpoints, True)
    #     if save_fig:
    #         if not os.path.isdir(save_dir):
    #             os.makedirs(save_dir)
    #         cv2.imwrite(f'{save_dir}/{image_path}_reproject_corners.png', img_draw)
    #     if vis:
    #         cv2.imshow('img_draw', img_draw)
    #         cv2.waitKey(0)
    # ===============================================================

    return error    

# Find chessboard corners in a grayscale image
def _find_chessboard_corners(gray: np.array, XX: int, YY:int, flags: int, criteria: Tuple[int, int ,float], winsize: Tuple[int, int]):

    # the 3rd params "cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK+cv2.CALIB_CB_NORMALIZE_IMAGE" can help to improve detection quality
    ret, corners = cv2.findChessboardCorners(gray, (XX, YY), cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK+cv2.CALIB_CB_NORMALIZE_IMAGE)
    if ret:
        corners2 = cv2.cornerSubPix(gray, corners, winsize, (-1, -1), criteria)
        return True, corners2
    else:
        return False, []

# detect corners in images in image_paths
def get_chessboard_corners(image_paths, XX, YY):
    imgpoints = []
    image_size = None
    for i in range(0, len(image_paths)):
        image_path = image_paths[i]

        img = cv2.imread(image_path)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        if image_size is None:
            image_size = gray.shape[::-1]
        
        find_chessboard_corners_flags= cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK+cv2.CALIB_CB_NORMALIZE_IMAGE
        criteria=(cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 30, 0.001)
        winsize=(11, 11)
        find_corners_ret, corners = _find_chessboard_corners(gray, XX, YY, find_chessboard_corners_flags, criteria, winsize)

        # corners: order in row by row, and left to right in each row
        if find_corners_ret:
            imgpoints.append(corners)
        else:
            print(f"Can not find checkerboard corners of {image_path}, Skip.")

    return imgpoints

# calculate the intrinsic and extrinsic parameters of camera, when we don't know the intrinsic parameters
def calibrate_camera(image_path_list: List[str], corners_3d_list: List[List[float]], 
                  XX: int, YY: int,
                  find_chessboard_corners_flags: int = cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK+cv2.CALIB_CB_NORMALIZE_IMAGE,
                  corner_refine_criteria=(cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 30, 0.001), 
                  corner_refine_winsize=(11, 11),
                  verbose=True) \
    -> Tuple[bool, np.array, np.array, np.array, np.array]:

    imgpoints = []
    objpoints = []
    image_size = None

    # detect corners in images in image_path_list
    for i in range(0, len(image_path_list)):
        
        image_path = image_path_list[i]
        img = cv2.imread(image_path)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        if image_size is None:
            image_size = gray.shape[::-1]

        find_corners_ret, corners = _find_chessboard_corners(gray, XX, YY, find_chessboard_corners_flags, corner_refine_criteria, corner_refine_winsize)

        if find_corners_ret:

            imgpoints.append(corners)   # shape = num*(88,1,2)
            objp = corners_3d_list[i]
            objpoints.append(objp)
        else:
            print(f"Can not find checkerboard corners of {image_path}, Skip.")
    
    # calculate the intrinsic parameters and extrinsic parameters(3x1 rotation vecs and 3x1 translation vecs)
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, image_size, None, None)

    return ret, mtx, dist, rvecs, tvecs

# calculate the intrinsic params
def _calibrate_intrinsic_camera_parameters(
    image_paths: Sequence[str], 
    XX: int, 
    YY: int, 
    L: float,
    verbose: bool=True
) -> Tuple[ArrayLike, ArrayLike]:
    
    # generate 3D positions of corners
    objpoints = get_objpoints(len(image_paths), XX, YY, L)

    # calibrate the camera, only use the intrinsic parameters
    ret, mtx, dist, rvecs_target2cam, tvecs_target2cam = calibrate_camera(image_paths, objpoints, XX, YY, verbose=verbose)
    return mtx, dist

# calculate the extrinsic params
def _calibrate_extrinsic_camera_parameters(extrinsic_calib_image_paths, XX, YY, L, verbose=True):

    objpoints = get_objpoints(len(extrinsic_calib_image_paths), XX, YY, L)

    # calibrate the camera, only use the extrinsic parameters
    ret, _, _, rvecs_target2cam, tvecs_target2cam = calibrate_camera(extrinsic_calib_image_paths, objpoints, XX, YY, verbose)
    return rvecs_target2cam, tvecs_target2cam

# count the number of images in the calibration data directory
def count_calibration_images(CALIBRATION_DATA_DIR):
    
    image_extensions = ['*.jpg', '*.jpeg', '*.png', '*.bmp', '*.tiff', '*.tif']
    image_paths = []
    for ext in image_extensions:
        image_paths.extend(glob(os.path.join(CALIBRATION_DATA_DIR, ext)))
        # image_paths.extend(glob(os.path.join(CALIBRATION_DATA_DIR, ext.upper())))
    
    return len(image_paths)

# #=================================Specific functions of mathematics=====================================
# calculate matrix from rpy angles (radians)
def rpy_to_matrix(coords):
        coords = np.asanyarray(coords, dtype=np.float64)
        c3, c2, c1 = np.cos(coords)
        s3, s2, s1 = np.sin(coords)

        return np.array([
            [c1 * c2, (c1 * s2 * s3) - (c3 * s1), (s1 * s3) + (c1 * c3 * s2)],
            [c2 * s1, (c1 * c3) + (s1 * s2 * s3), (c3 * s1 * s2) - (c1 * s3)],
            [-s2, c2 * s3, c2 * c3]
        ], dtype=np.float64)

# calculate matrix from xyz positions (meters) and rpy angles(radians)
def xyz_rpy_to_matrix(xyz_rpy):

    R = rpy_to_matrix(xyz_rpy[3:])  # 3x3
    t = xyz_rpy[:3]  # 3x1
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = R
    matrix[:3, 3] = t
    return matrix

# calculate the inverse of a transformation matrix
def inverse_transform_matrix(T):
    
    R = T[:3, :3]
    t = T[:3, 3]

    R_inv = R.T
    t_inv = -np.dot(R_inv, t)

    T_inv = np.identity(4)
    T_inv[:3, :3] = R_inv
    T_inv[:3, 3] = t_inv

    return T_inv

# calculate the xyz positions (meters) and rpy angles(radians) from 4x4 matrix
def matrix_to_xyz_rpy(matrix):

    x, y, z = matrix[:3, 3]
    
    R = matrix[:3, :3]
    
    # calculate Euler angles
    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    
    singular = sy < 1e-6
    
    if not singular:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0
    
    return x, y, z, roll, pitch, yaw

# #=================================Specific functions of intrinsic calibration===============================================
# load the images used for intrinsic calibration 
def load_intrinsic_calib_images(INTRINSIC_CALIB_DATA_DIR, image_idxs=None):
    
    image_extensions = ['*.jpg', '*.jpeg', '*.png', '*.bmp', '*.tiff', '*.tif']
    image_paths = []
    for ext in image_extensions:
        image_paths.extend(glob(os.path.join(INTRINSIC_CALIB_DATA_DIR, ext)))
        # image_paths.extend(glob(os.path.join(INTRINSIC_CALIB_DATA_DIR, ext.upper())))
    
    image_paths.sort(key=lambda x: int(os.path.split(x)[-1].split('.')[0]))
    calib_image_paths = image_paths if image_idxs is None else np.array(image_paths)[image_idxs].flatten()
    return calib_image_paths

# calculate the intrinsic parameters
def calibrate_intrinsic_camera_parameters(intrinsic_calib_image_paths, XX, YY, L, verbose=False):
    
    if XX is None or YY is None or L is None:
        raise ValueError("Checkerboard size has not been loaded yet.")
    
    if intrinsic_calib_image_paths is None:
        raise ValueError("Intrinsic calibration images have not been loaded yet.")
    
    camera_matrix, distortion_coefficients = _calibrate_intrinsic_camera_parameters(
        intrinsic_calib_image_paths, XX, YY, L, verbose=verbose)
    
    return camera_matrix, distortion_coefficients

# save the intrinsic parameters after calculation
def save_intrinsic_params(camera_matrix, distortion_coefficients, save_dir):
    
    if not os.path.isdir(save_dir):
        os.makedirs(save_dir)
    
    mtx_dict = {"camera_matrix": camera_matrix.tolist()}
    with open(f'{save_dir}/mtx.json', 'w', encoding='utf-8') as f:
        json.dump(mtx_dict, f, indent=4, ensure_ascii=False)
    
    dist_dict = {"distortion_coefficients": distortion_coefficients.tolist()}
    with open(f'{save_dir}/dist.json', 'w', encoding='utf-8') as f:
        json.dump(dist_dict, f, indent=4, ensure_ascii=False)
    
    print(f"camera intrinsic parameters have been saved to: {save_dir}")

# load the intrinsic params 
def load_intrinsic_params(save_dir):
    
    camera_matrix_path = f'{save_dir}/mtx.json'
    if not os.path.isfile(camera_matrix_path):
        raise ValueError(f"Camera intrinsic parameters have not been saved at {save_dir} yet.")
    distortion_coefficients_path = f'{save_dir}/dist.json'
    if not os.path.isfile(distortion_coefficients_path):
        raise ValueError(f"Camera distortion coefficients have not been saved at {save_dir} yet.")
    
    with open(camera_matrix_path, 'r', encoding='utf-8') as f:
        mtx_dict = json.load(f)
        camera_matrix = np.array(mtx_dict["camera_matrix"])
    
    with open(distortion_coefficients_path, 'r', encoding='utf-8') as f:
        dist_dict = json.load(f)
        distortion_coefficients = np.array(dist_dict["distortion_coefficients"])
    
    return camera_matrix, distortion_coefficients

# #=================================Specific functions of extrinsic calibration===============================================
# load the intrinsic parameters for extrinsic calibration
def load_camera_intrinsics(save_dir):
   
    mtx, dist = load_intrinsic_params(save_dir)
    return mtx, dist

# load the calibration images for extrinsic calibration
def load_calibration_images(HANDEYE_CALIB_DATA_DIR, selected_idxs=None):
    
    image_extensions = ['*.jpg', '*.jpeg', '*.png', '*.bmp', '*.tiff', '*.tif']
    extrinsic_calib_image_paths = []
    for ext in image_extensions:
        extrinsic_calib_image_paths.extend(glob(os.path.join(HANDEYE_CALIB_DATA_DIR, ext)))
        # extrinsic_calib_image_paths.extend(glob(os.path.join(HANDEYE_CALIB_DATA_DIR, ext.upper())))
    
    extrinsic_calib_image_paths.sort(key=lambda x: int(os.path.split(x)[-1].split('.')[0]))
    calib_image_paths = extrinsic_calib_image_paths if selected_idxs is None else np.array(extrinsic_calib_image_paths)[selected_idxs].flatten()
    print(f"Successfully load calibration {len(calib_image_paths)} images.")
    return calib_image_paths

# load the end effector poses for extrinsic calibration
def load_calibration_images_pose(HANDEYE_CALIB_DATA_DIR, selected_idxs=None):
    
    end2base_Ms = []
    end_pose_json_paths = glob(f"{HANDEYE_CALIB_DATA_DIR}/*.json")
    end_pose_json_paths = sorted(end_pose_json_paths, key=lambda x: int(os.path.split(x)[-1].split('.')[0]))
    # if there are some poor images need to be deleted, its corresponding poses need also to be deleted.
    selected_end_pose_json_paths = end_pose_json_paths if selected_idxs is None else np.array(end_pose_json_paths)[selected_idxs].flatten()
    
    for path in selected_end_pose_json_paths:
        with open(path, 'r', encoding='utf-8') as f:
            pose_data = json.load(f)
            end_xyzrpy_dict = pose_data["end_xyzrpy"]
            xyzrpy = np.array([
                end_xyzrpy_dict["x"],
                end_xyzrpy_dict["y"], 
                end_xyzrpy_dict["z"],
                end_xyzrpy_dict["rx"],
                end_xyzrpy_dict["ry"],
                end_xyzrpy_dict["rz"]
            ])
        
        end2base_M = xyz_rpy_to_matrix(xyzrpy)
        end2base_Ms.append(end2base_M)

    base2end_Ms = np.array([inverse_transform_matrix(x) for x in end2base_Ms])
    print(f"Successfully load {len(end2base_Ms)} pose data end2base.")
    return np.array(base2end_Ms), np.array(end2base_Ms)

# the main function of eye in hand calibration
def calibrate_camera_eyeinhand(calib_image_paths, end2base_Ms, XX, YY, L, mtx, dist, verbose=True):

    # check if the needed data has been loaded
    if mtx is None or dist is None:
        raise ValueError("Camera intrinsic parameters have not been loaded yet.")
    
    if calib_image_paths is None:
        raise ValueError("Extrinsic calibration images have not been loaded yet.")
    
    # calculate chessboard2cam accoring to multiple images when the intrinsic params are known  
    rvecs_target2cam, tvecs_target2cam = _calibrate_extrinsic_camera_parameters(calib_image_paths, 
                                                                              XX, YY, L, verbose)

    # get the rotation and translation
    end2base_Rs = end2base_Ms[:, :3, :3]
    end2base_ts = end2base_Ms[:, :3, 3]

    # eye in hand calibration 
    cam2end_R, cam2end_t = cv2.calibrateHandEye(end2base_Rs, end2base_ts, rvecs_target2cam, tvecs_target2cam, cv2.CALIB_HAND_EYE_HORAUD)
    
    cam2end_4x4 = np.eye(4)
    cam2end_4x4[:3, :3] = cam2end_R
    cam2end_4x4[:3, 3] = cam2end_t[:, 0]
    
    print("Eye-in-hand calibration completed successfully.")
    print(f"The transformation matrix cam2end:\n{cam2end_4x4}")

    return cam2end_R, cam2end_t, cam2end_4x4, rvecs_target2cam, tvecs_target2cam

# calculate the reprojection error from target2cam obtained from images
def calculate_reprojection_errors_intrinsics(calib_image_paths, rvecs_target2cam, tvecs_target2cam, mtx, dist, XX, YY, L, vis=False):
    
    errors = []
    print("Calculating intrinsic reprojection errors...")
    for i in range(len(calib_image_paths)):
        error = calculate_single_image_reprojection_error(
            calib_image_paths[i], 
            rvecs_target2cam[i], tvecs_target2cam[i], mtx, dist, XX, YY, L, vis=vis)
        errors.append(error)
   
    return np.array(errors)

# calculate the reprojection error from target2cam obtained from eye-in-hand calibration
def calculate_reprojection_errors_extrinsics(calib_image_paths, rvecs_target2cam, tvecs_target2cam, end2base_Ms, base2end_Ms, cam2end_4x4, mtx, dist, XX, YY, L, vis=False, vis_result_save_dir = None):
    
    
    end2cam_4x4 = np.linalg.inv(cam2end_4x4)
    errors = []
    target2base_Ms = []
    print("Calculating extrinsic reprojection errors...")

    for i in range(len(end2base_Ms)):

        # target2cam obtained from images, 4x4
        target2cam_4x4 = xyz_rpy_to_matrix(tvecs_target2cam[i].flatten().tolist() + rvecs_target2cam[i].flatten().tolist())

        target2base_4x4 = end2base_Ms[i] @ cam2end_4x4 @ target2cam_4x4
        target2base_Ms.append(target2base_4x4)

        eyeinhand_target2cam_4x4 = end2cam_4x4 @ base2end_Ms[i] @ target2base_4x4

        error = calculate_single_image_reprojection_error(
            calib_image_paths[i], 
            eyeinhand_target2cam_4x4[:3, :3], eyeinhand_target2cam_4x4[:3, 3], mtx, dist, XX, YY, L, vis=vis)
        errors.append(error)

        # decide whether to visualize the results
        if vis:
            # ===============================================================================================
            objpoints = get_objpoints(1, XX, YY, L)
            reprojected_imgpoints, _ = cv2.projectPoints(objpoints[0], eyeinhand_target2cam_4x4[:3, :3], eyeinhand_target2cam_4x4[:3, 3], mtx, dist)
            
            img = cv2.imread(calib_image_paths[i])
            img_draw = img.copy()
            
            # detect
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            ret, detected_corners = cv2.findChessboardCorners(gray, (XX, YY), None)
            
            if ret:
                # improve the corners
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                detected_corners = cv2.cornerSubPix(gray, detected_corners, (11, 11), (-1, -1), criteria)
                
                # real corners
                for corner in detected_corners:
                    cv2.circle(img_draw, tuple(corner[0].astype(int)), 8, (0, 255, 0), 2)
                
                # predict corners
                for point in reprojected_imgpoints:
                    center = tuple(point[0].astype(int))
                    cv2.drawMarker(img_draw, center, (0, 0, 255), cv2.MARKER_CROSS, 12, 2)
                
                cv2.putText(img_draw, "Green: Detected corners", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(img_draw, "Red: Reprojected corners", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # save the results
            if not os.path.isdir(vis_result_save_dir):
                os.makedirs(vis_result_save_dir)
            image_name = os.path.basename(calib_image_paths[i]).split('.')[0]
            cv2.imwrite(f'{vis_result_save_dir}/eyeinhand_reproject_{image_name}.png', img_draw)
            # ====================================================================================================


    return np.array(errors), target2base_Ms

# #=================================Specific functions of optimization===============================================
# calculate target2cam from target2base
def calculate_target2cam_4x4_list_target2base(x, y, z, roll, pitch, yaw, cam2end_4x4, end2base_Ms):
    
    target2cam_4x4_list = []

    target2base_4x4 = xyz_rpy_to_matrix([x, y, z, roll, pitch, yaw])

    end2cam_4x4 = np.linalg.inv(cam2end_4x4)
    
    for end2base_M in end2base_Ms:
        base2end_4x4 = np.linalg.inv(end2base_M)
        target2cam_4x4 = end2cam_4x4 @ base2end_4x4 @ target2base_4x4
        target2cam_4x4_list.append(target2cam_4x4)   

    return target2cam_4x4_list

# calculate reprojection error of target2cam corresponding to each target2base, and use for objective function
def objective_function_target2base(x, objpoints, imgpoints, cam2end_4x4, end2base_Ms, mtx, dist, image_paths, XX, YY, grad=None):
    
    # 拆分变量target2base的六个参数
    x_, y_, z_, roll_, pitch_, yaw_ = x[0], x[1], x[2], x[3], x[4], x[5]
    
    # 根据优化变量计算target2cam
    target2cam_4x4_list = calculate_target2cam_4x4_list_target2base(x_, y_, z_, roll_, pitch_, yaw_, cam2end_4x4, end2base_Ms)
    rvecs_v2 = np.array(target2cam_4x4_list)[:, :3, :3]
    tvecs_v2 = np.array(target2cam_4x4_list)[:, :3, 3]
    
    # 计算当前的重投影误差的平均值
    error = calculate_reproject_error_fast(
        imgpoints=imgpoints,
        objpoints=objpoints,
        rvecs_target2cam=rvecs_v2, 
        tvecs_target2cam=tvecs_v2,
        mtx=mtx,
        dist=dist,
        image_paths=image_paths, 
        XX=XX,
        YY=YY,
        vis=False,
        save_fig=False)
    
    return error

# calculate target2cam from cam2end
def calculate_target2cam_list_on_cam2end(cam2end_x, cam2end_y, cam2end_z, cam2end_roll, cam2end_pitch, cam2end_yaw, end2base_Ms, target2base_4x4):

    cam2end_4x4 = xyz_rpy_to_matrix([cam2end_x, cam2end_y, cam2end_z, cam2end_roll, cam2end_pitch, cam2end_yaw])

    eyehand_target2cam_4x4_list = []
    end2cam_4x4 = np.linalg.inv(cam2end_4x4)

    for idx in range(len(end2base_Ms)):
        base2end_Ms = np.linalg.inv(end2base_Ms[idx])
        eyehand_target2cam_4x4 = end2cam_4x4 @ base2end_Ms @ target2base_4x4
        eyehand_target2cam_4x4_list.append(eyehand_target2cam_4x4)
    return eyehand_target2cam_4x4_list

# calculate reprojection error of target2cam corresponding to each cam2end, and use for objective function
def objective_function_cam2end(x, objpoints, imgpoints, end2base_Ms, target2base_4x4, mtx, dist, image_paths, XX, YY, grad=None):

    x_, y_, z_, roll_, pitch_, yaw_ = x[0], x[1], x[2], x[3], x[4], x[5]
    
    target2cam_4x4_list = calculate_target2cam_list_on_cam2end(x_, y_, z_, roll_, pitch_, yaw_, end2base_Ms, target2base_4x4)
    rvecs_v2 = np.array(target2cam_4x4_list)[:, :3, :3]
    tvecs_v2 = np.array(target2cam_4x4_list)[:, :3, 3]
    
    error = calculate_reproject_error_fast(
        imgpoints=imgpoints,
        objpoints=objpoints,
        image_paths=image_paths, 
        rvecs_target2cam=rvecs_v2, 
        tvecs_target2cam=tvecs_v2,
        mtx=mtx,
        dist=dist,
        XX=XX,
        YY=YY,
        vis=False,
        save_fig=False)
    
    return error

# optimization function of target2base
def optimize_target2base(init_target2cam_image_idx, rvecs_target2cam, tvecs_target2cam, end2base_Ms, cam2end_4x4, objpoints, imgpoints, mtx, dist, image_paths, XX, YY, opt_ftol_rel):

    # optimization parameters
    OPT_FTOL_REL = opt_ftol_rel

    x_target2cam = tvecs_target2cam[init_target2cam_image_idx][0].item()
    y_target2cam = tvecs_target2cam[init_target2cam_image_idx][1].item()
    z_target2cam = tvecs_target2cam[init_target2cam_image_idx][2].item()
    roll_target2cam = rvecs_target2cam[init_target2cam_image_idx][0].item()
    pitch_target2cam = rvecs_target2cam[init_target2cam_image_idx][1].item()
    yaw_target2cam = rvecs_target2cam[init_target2cam_image_idx][2].item()
                                
    target2cam_4x4 = xyz_rpy_to_matrix([x_target2cam, y_target2cam, z_target2cam, roll_target2cam, pitch_target2cam, yaw_target2cam])
    target2base_4x4 = end2base_Ms[init_target2cam_image_idx] @ cam2end_4x4 @ target2cam_4x4

    x, y, z, roll, pitch, yaw = matrix_to_xyz_rpy(target2base_4x4)

    # optimization algorithm
    opt = nlopt.opt(nlopt.LN_NELDERMEAD, 6)  # 6 is the number of variables

    def objective(x, grad):
        return objective_function_target2base(x, objpoints, imgpoints, cam2end_4x4, end2base_Ms, mtx, dist, image_paths, XX, YY, grad)
    # set the objective function
    opt.set_min_objective(objective)

    # set the bounds
    lower_bounds = [-100, -100, -100, -100, -100, -100]
    upper_bounds = [100, 100, 100, 100, 100, 100]
    opt.set_lower_bounds(lower_bounds)
    opt.set_upper_bounds(upper_bounds)

    # set the stopping criteria
    opt.set_ftol_rel(OPT_FTOL_REL)

    # set the initial values
    x_initial = [x, y, z, roll, pitch, yaw]
    print(f"Target2base X initial: {x_initial}")

    # Run the optimizer
    x_optimal = opt.optimize(x_initial)
    optimal_value = opt.last_optimum_value()
    print(f"Target2base Optimal variables: {x_optimal}")
    print(f"Target2base Optimal loss: {optimal_value}")

    x_opt, y_opt, z_opt, roll_opt, pitch_opt, yaw_opt = x_optimal
    
    # update the target2base
    optimized_target2base_4x4 = xyz_rpy_to_matrix([x_opt, y_opt, z_opt, roll_opt, pitch_opt, yaw_opt])
    return optimized_target2base_4x4

# optimization function of cam2end
def optimize_cam2end(init_target2cam_image_idx, rvecs_target2cam, tvecs_target2cam, end2base_Ms, target2base_4x4, objpoints, imgpoints, mtx, dist, image_paths, XX, YY, opt_ftol_rel):

    # optimization parameters
    OPT_FTOL_REL = opt_ftol_rel
    
    # init pos    
    x_target2cam = tvecs_target2cam[init_target2cam_image_idx][0].item()
    y_target2cam = tvecs_target2cam[init_target2cam_image_idx][1].item()
    z_target2cam = tvecs_target2cam[init_target2cam_image_idx][2].item()
    roll_target2cam = rvecs_target2cam[init_target2cam_image_idx][0].item()
    pitch_target2cam = rvecs_target2cam[init_target2cam_image_idx][1].item()
    yaw_target2cam = rvecs_target2cam[init_target2cam_image_idx][2].item()
                                
    target2cam_4x4 = xyz_rpy_to_matrix([x_target2cam, y_target2cam, z_target2cam, roll_target2cam, pitch_target2cam, yaw_target2cam])
    
    # init cam2end by optimized target2base
    cam2target_4x4 = np.linalg.inv(target2cam_4x4)
    base2end_4x4 = np.linalg.inv(end2base_Ms[init_target2cam_image_idx])
    cam2end_4x4 = base2end_4x4 @ target2base_4x4 @ cam2target_4x4
    x, y, z, roll, pitch, yaw = matrix_to_xyz_rpy(cam2end_4x4)
            
    # Choose the NL_NELDERMEAD algorithm (gradient-free)
    opt = nlopt.opt(nlopt.LN_NELDERMEAD, 6)  # 6 is the number of variables

    # Set the objective function
    def objective(x, grad):
        return objective_function_cam2end(x, objpoints, imgpoints, end2base_Ms, target2base_4x4, mtx, dist, image_paths, XX, YY, grad)
    
    opt.set_min_objective(objective)

    # Set lower and upper bounds for each variable
    lower_bounds = [-100, -100, -100, -100, -100, -100]
    upper_bounds = [100, 100, 100, 100, 100, 100]
    opt.set_lower_bounds(lower_bounds)
    opt.set_upper_bounds(upper_bounds)

    # Set stopping criteria 
    opt.set_ftol_rel(OPT_FTOL_REL)

    # set the initial values
    x_initial = [x, y, z, roll, pitch, yaw]
    print(f"Cam2end X initial: {x_initial}")

    # Run the optimizer
    x_optimal = opt.optimize(x_initial)
    optimal_value = opt.last_optimum_value()

    print(f"Cam2end Optimal variables: {x_optimal}")
    print(f"Cam2end Optimal loss: {optimal_value}")
    
    x_opt, y_opt, z_opt, roll_opt, pitch_opt, yaw_opt = x_optimal
    
    optimized_cam2end_4x4 = xyz_rpy_to_matrix([x_opt, y_opt, z_opt, roll_opt, pitch_opt, yaw_opt])
    return optimized_cam2end_4x4

# the main function of optimization process
def optimize_eyeinhand_calibration(calib_image_paths, rvecs_target2cam, tvecs_target2cam, end2base_Ms, base2end_Ms, cam2end_4x4, mtx, dist, XX, YY, L, init_target2cam_image_idx, iterations, opt_ftol_rel):
    
    # pre-calculate the object points and image points
    num_images = len(calib_image_paths)
    objpoints = get_objpoints(num_images, XX, YY, L)
    imgpoints = get_chessboard_corners(calib_image_paths, XX, YY)

    # initialize the values needed
    current_cam2end_4x4 = cam2end_4x4.copy()
    current_target2base_4x4 = None
    
    for i in range(iterations):
        print(f"========== Iteration {i+1} ==========")

        print("------------OPT target2base ---------------------")
        current_target2base_4x4 = optimize_target2base(
            init_target2cam_image_idx, rvecs_target2cam, tvecs_target2cam, 
            end2base_Ms, current_cam2end_4x4, objpoints, imgpoints, 
            mtx, dist, calib_image_paths, XX, YY, opt_ftol_rel)

        print("------------OPT cam2end ---------------------")
        current_cam2end_4x4 = optimize_cam2end(
            init_target2cam_image_idx, rvecs_target2cam, tvecs_target2cam, 
            end2base_Ms, current_target2base_4x4, objpoints, imgpoints, 
            mtx, dist, calib_image_paths, XX, YY, opt_ftol_rel)

    return current_cam2end_4x4, current_target2base_4x4

# calculate the reprojection errors after optimization
def calculate_optimized_reprojection_errors(calib_image_paths, base2end_Ms, cam2end_4x4, target2base_4x4, mtx, dist, XX, YY, L, vis=False, vis_result_save_dir=None):
    
    end2cam_4x4 = np.linalg.inv(cam2end_4x4)
    errors = []

    for i in range(len(base2end_Ms)):
        eyehand_target2cam_4x4 = end2cam_4x4 @ base2end_Ms[i] @ target2base_4x4
    
        error = calculate_single_image_reprojection_error(
            calib_image_paths[i], 
            eyehand_target2cam_4x4[:3, :3], eyehand_target2cam_4x4[:3, 3], mtx, dist, XX, YY, L, vis=vis)
        errors.append(error)

        # decide whether to visualize the results
        if vis:

            objpoints = get_objpoints(1, XX, YY, L)
            reprojected_imgpoints, _ = cv2.projectPoints(objpoints[0], eyehand_target2cam_4x4[:3, :3], eyehand_target2cam_4x4[:3, 3], mtx, dist)

            img = cv2.imread(calib_image_paths[i])
            img_draw = img.copy()
            
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            ret, detected_corners = cv2.findChessboardCorners(gray, (XX, YY), None)
            
            if ret:
                # improve the corners
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                detected_corners = cv2.cornerSubPix(gray, detected_corners, (11, 11), (-1, -1), criteria)
                
                # real corners
                for corner in detected_corners:
                    cv2.circle(img_draw, tuple(corner[0].astype(int)), 8, (0, 255, 0), 2)
                
                # predict corners
                for point in reprojected_imgpoints:
                    center = tuple(point[0].astype(int))
                    cv2.drawMarker(img_draw, center, (0, 0, 255), cv2.MARKER_CROSS, 12, 2)
                
                cv2.putText(img_draw, "Green: Detected corners", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(img_draw, "Red: Reprojected corners", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # save the reprojected result image
            if vis_result_save_dir and not os.path.isdir(vis_result_save_dir):
                os.makedirs(vis_result_save_dir)
            if vis_result_save_dir:
                image_name = os.path.basename(calib_image_paths[i]).split('.')[0]
                cv2.imwrite(f'{vis_result_save_dir}/optimized_eyeinhand_reproject_{image_name}.png', img_draw)
    
    return np.array(errors)

# save the calibration results after optimizaton
def save_optimization_results(cam2end_4x4, target2base_4x4, rvecs_target2cam, tvecs_target2cam, mtx, dist, save_dir):
    
    if not os.path.isdir(save_dir):
        os.makedirs(save_dir)

    target2cam_matrices = []
    
    # calculate target2cam based on rvecs_target2cam, tvecs_target2cam
    for i in range(len(rvecs_target2cam)):
        rvec = rvecs_target2cam[i].flatten()
        tvec = tvecs_target2cam[i].flatten()
        target2cam_xyz_rpy = [tvec[0], tvec[1], tvec[2], rvec[0], rvec[1], rvec[2]]
        target2cam_4x4 = xyz_rpy_to_matrix(target2cam_xyz_rpy)
        
        target2cam_matrices.append(target2cam_4x4.tolist())

    calibration_results = {
        "camera_intrinsics": {
            "camera_matrix": mtx.tolist(),
            "distortion_coefficients": dist.tolist()
        },
        "eye_in_hand_calibration_results": {
            "optimized_cam2end_matrix": cam2end_4x4.tolist(),
            "optimized_target2base_matrix": target2base_4x4.tolist(),
        },
        "camera_extrinsic": {
            "chessboard2cam_matrices": target2cam_matrices,
        }
    }
    
    json_file_path = f"{save_dir}/calibration_results.json"
    with open(json_file_path, 'w', encoding='utf-8') as f:
        json.dump(calibration_results, f, indent=4, ensure_ascii=False)
    
    print(f"calibration results have been saved to: {json_file_path}")

# #===================================main function======================================================
def main(CALIBRATION_DATA_DIR, CALIBRATION_PARAMETER_OUTPUT_DIR, XX, YY, L, REPROJECTION_RESULT_OUTPUT_DIR = None):

    # 1. set chessboard parameters and automatically count images
    # XX = 11 # corners along x-axis
    # YY = 8  # corners along y-axis
    # L = 0.02 # square size in meters
    IMAGE_NUM = count_calibration_images(CALIBRATION_DATA_DIR)

    # 2. set camera parameters, data directory and result output directory
    os.makedirs(CALIBRATION_PARAMETER_OUTPUT_DIR, exist_ok=True)
    
    vis = REPROJECTION_RESULT_OUTPUT_DIR is not None
    if vis:
        os.makedirs(REPROJECTION_RESULT_OUTPUT_DIR, exist_ok=True)

    print(f"Calibration images folder: {CALIBRATION_DATA_DIR}")
    print(f"Calibration params output folder: {CALIBRATION_PARAMETER_OUTPUT_DIR}")
    print(f"Chessboard size: {XX} x {YY}, square size: {L} m")
    print(f"Visualize reprojection: {vis}")

    # 3. calibrate camera intrinsic parameters (if needed)
    try:
        calib_image_paths = load_intrinsic_calib_images(CALIBRATION_DATA_DIR)
        camera_matrix, distortion_coefficients = calibrate_intrinsic_camera_parameters(calib_image_paths, XX, YY, L, verbose=True)
        save_intrinsic_params(camera_matrix, distortion_coefficients, CALIBRATION_PARAMETER_OUTPUT_DIR)
        # loaded_mtx, loaded_dist = load_intrinsic_params(CALIBRATION_PARAMETER_OUTPUT_DIR)
    except Exception as e:
        print(f"Error in intrinsic calibration : {e}")

    # 4. calibrate camera extrinsic parameters
    # 4.1 inspect extrinsic calibration images
    extrinsic_calib_image_idxs = np.arange(0, IMAGE_NUM)
    excluded_image_idxs = []    # if some images need to be excluded, enter the index of it in here
    extrinsic_calib_image_idxs = np.delete(extrinsic_calib_image_idxs, excluded_image_idxs)

    # 4.2 load camera intrinsics parameters, calibration images and corresponding poses
    mtx, dist = load_camera_intrinsics(CALIBRATION_PARAMETER_OUTPUT_DIR)
    calib_image_paths = load_calibration_images(CALIBRATION_DATA_DIR, extrinsic_calib_image_idxs)
    base2end_Ms, end2base_Ms = load_calibration_images_pose(CALIBRATION_DATA_DIR, extrinsic_calib_image_idxs)

    # 4.3 calibrate eye-in-hand camera extrinsic parameters
    cam2end_R, cam2end_t, cam2end_4x4, rvecs_target2cam, tvecs_target2cam = calibrate_camera_eyeinhand(calib_image_paths, end2base_Ms, XX, YY, L, mtx, dist, verbose=True)

    # 4.4 calculate the reprojection error based on calibrated matrix target2cam
    intrinsic_calib_errors = calculate_reprojection_errors_intrinsics(calib_image_paths, rvecs_target2cam, tvecs_target2cam, mtx, dist, XX, YY, L, vis=False)
    
    # # if find errors > 1, delete thses images with poor quality, redo the process of 4.2 and 4.3)
    # extrinsic_calib_image_idxs = extrinsic_calib_image_idxs[intrinsic_calib_errors<1]
    # calib_image_paths = load_calibration_images(CALIBRATION_DATA_DIR, extrinsic_calib_image_idxs)
    # base2end_Ms, end2base_Ms = load_calibration_images_pose(CALIBRATION_DATA_DIR, extrinsic_calib_image_idxs)
    # cam2end_R, cam2end_t, cam2end_4x4, rvecs_target2cam, tvecs_target2cam = calibrate_camera_eyeinhand(calib_image_paths, end2base_Ms, XX, YY, L, mtx, dist, verbose=True)

    # 4.5 calculate the reprojection error based on eye-in-hand calibration results
    extrinsic_calib_errors, target2base_Ms = calculate_reprojection_errors_extrinsics(calib_image_paths, rvecs_target2cam, tvecs_target2cam, end2base_Ms, base2end_Ms, cam2end_4x4, mtx, dist, XX, YY, L, vis, vis_result_save_dir=REPROJECTION_RESULT_OUTPUT_DIR)

    # 5. optimize the extrinsic calibration parameters

    # 5.1 set the initial index of optimization (the image with the min reprojection error) and optimization parameters
    OPT_ITERATIONS = 5     # number of optimization iterations
    OPT_FTOL_REL = 1e-6     # relative tolerance for optimization
    argmin_error_idx = np.argmin(extrinsic_calib_errors)
    print(f'argmin_error_idx: {argmin_error_idx}')

    # # initial matrix results(used to check the change of optimization results)
    # init_cam2end_4x4 = cam2end_4x4.copy()
    # x_target2cam = tvecs_target2cam[argmin_error_idx][0].item()
    # y_target2cam = tvecs_target2cam[argmin_error_idx][1].item()
    # z_target2cam = tvecs_target2cam[argmin_error_idx][2].item()
    # roll_target2cam = rvecs_target2cam[argmin_error_idx][0].item()
    # pitch_target2cam = rvecs_target2cam[argmin_error_idx][1].item()
    # yaw_target2cam = rvecs_target2cam[argmin_error_idx][2].item()
    # target2cam_4x4 = xyz_rpy_to_matrix([x_target2cam, y_target2cam, z_target2cam, roll_target2cam, pitch_target2cam, yaw_target2cam])
    # init_target2base_4x4 = end2base_Ms[argmin_error_idx] @ cam2end_4x4 @ target2cam_4x4
    
    # 5.2 start the optimization loop
    optimized_cam2end_4x4, optimized_target2base_4x4 = optimize_eyeinhand_calibration(calib_image_paths, rvecs_target2cam, tvecs_target2cam, end2base_Ms, end2base_Ms, cam2end_4x4, mtx, dist, XX, YY, L, argmin_error_idx, iterations=OPT_ITERATIONS, opt_ftol_rel=OPT_FTOL_REL)

    # 5.3 calculate the reprojection error based on eye-in-hand calibration results after optimization
    optimized_errors = calculate_optimized_reprojection_errors(calib_image_paths, base2end_Ms, optimized_cam2end_4x4, optimized_target2base_4x4, mtx, dist, XX, YY, L, vis, vis_result_save_dir=REPROJECTION_RESULT_OUTPUT_DIR)

    # 5.4 save optimization results
    save_optimization_results(optimized_cam2end_4x4, optimized_target2base_4x4, rvecs_target2cam, tvecs_target2cam, mtx, dist, CALIBRATION_PARAMETER_OUTPUT_DIR)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Eye-in-Hand Calibration")

    parser.add_argument("--calib_data_dir", required=True, help="Path to calibration images folder")
    parser.add_argument("--calib_out_dir", required=True, help="Path to output folder for calibration parameters")
    parser.add_argument("--xx", type=int, required=True, help="Number of corners along chessboard X axis")
    parser.add_argument("--yy", type=int, required=True, help="Number of corners along chessboard Y axis")
    parser.add_argument("--square_size", type=float, required=True, help="Size of one chessboard square in meters")
    parser.add_argument("--reproj_out_dir", required=False, help="(Optional) Output folder for reprojection visualization")

    args = parser.parse_args()

    main(args.calib_data_dir, args.calib_out_dir, args.xx, args.yy, args.square_size, args.reproj_out_dir)
