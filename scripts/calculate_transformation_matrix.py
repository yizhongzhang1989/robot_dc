#!/usr/bin/env python3
"""
计算correct2estimate变换矩阵
correct2estimate = estimated2base^(-1) * tcp2base
"""

import json
import numpy as np
from scipy.spatial.transform import Rotation

def rotation_vector_to_matrix(rx, ry, rz):
    """将旋转向量转换为旋转矩阵"""
    rotvec = np.array([rx, ry, rz])
    rotation = Rotation.from_rotvec(rotvec)
    return rotation.as_matrix()

def matrix_to_rotation_vector(R):
    """将旋转矩阵转换为旋转向量"""
    rotation = Rotation.from_matrix(R)
    return rotation.as_rotvec()

def load_correct_pose(file_path):
    """加载correct pose (tcp2base)"""
    with open(file_path, 'r') as f:
        data = json.load(f)
    
    # 构建tcp2base变换矩阵
    # 注意：这里假设correct_pose没有平移分量，只有旋转
    # 如果有平移分量，需要从数据中读取
    R = rotation_vector_to_matrix(data['correct_rx'], data['correct_ry'], data['correct_rz'])
    
    tcp2base = np.eye(4)
    tcp2base[:3, :3] = R
    # 如果correct_pose.json中有平移分量，在这里添加
    
    return tcp2base

def load_estimated_pose(file_path):
    """加载estimated2base变换矩阵"""
    with open(file_path, 'r') as f:
        data = json.load(f)
    
    # 读取transformation_matrix
    estimated2base = np.array(data['transformation_matrix'])
    
    return estimated2base

def calculate_correct2estimate(tcp2base, estimated2base):
    """
    计算correct2estimate变换矩阵（仅姿态）
    correct2estimate = estimated2base_R^(-1) * tcp2base_R
    """
    # 只提取旋转矩阵部分
    tcp2base_R = tcp2base[:3, :3]
    estimated2base_R = estimated2base[:3, :3]
    
    # 计算旋转变换
    estimated2base_R_inv = estimated2base_R.T  # 旋转矩阵的逆等于转置
    correct2estimate_R = estimated2base_R_inv @ tcp2base_R
    
    # 构建4x4变换矩阵（无平移）
    correct2estimate = np.eye(4)
    correct2estimate[:3, :3] = correct2estimate_R
    
    return correct2estimate

def save_transformation_matrix(matrix, file_path):
    """保存变换矩阵到JSON文件"""
    # 提取旋转矩阵和平移向量
    R = matrix[:3, :3]
    t = matrix[:3, 3]
    
    # 转换为旋转向量
    rotvec = matrix_to_rotation_vector(R)
    
    data = {
        "transformation_matrix": matrix.tolist(),
        "rotation_matrix": R.tolist(),
        "translation_vector": t.tolist(),
        "rotation_vector": {
            "rx": float(rotvec[0]),
            "ry": float(rotvec[1]),
            "rz": float(rotvec[2])
        },
        "pose_representation": {
            "x": float(t[0]),
            "y": float(t[1]),
            "z": float(t[2]),
            "rx": float(rotvec[0]),
            "ry": float(rotvec[1]),
            "rz": float(rotvec[2])
        },
        "description": "Rotation transformation from correct orientation to estimated orientation (only rotation, no translation)"
    }
    
    with open(file_path, 'w') as f:
        json.dump(data, f, indent=2)
    
    print(f"Transformation matrix saved to: {file_path}")

def main():
    # 文件路径
    correct_pose_file = '/home/a/Documents/robot_dc/temp/ur_test_data/correct_pose.json'
    estimated_pose_file = '/home/a/Documents/robot_dc/temp/ur_test_result/keypoint_coordinate_system.json'
    output_file = '/home/a/Documents/robot_dc/temp/ur_test_data/transformation_matrix.json'
    
    # 加载数据
    print("Loading correct pose (tcp2base)...")
    tcp2base = load_correct_pose(correct_pose_file)
    print("tcp2base matrix:")
    print(tcp2base)
    print()
    
    print("Loading estimated pose (estimated2base)...")
    estimated2base = load_estimated_pose(estimated_pose_file)
    print("estimated2base matrix:")
    print(estimated2base)
    print()
    
    # 计算correct2estimate
    print("Calculating correct2estimate...")
    correct2estimate = calculate_correct2estimate(tcp2base, estimated2base)
    print("correct2estimate matrix:")
    print(correct2estimate)
    print()
    
    # 保存结果
    save_transformation_matrix(correct2estimate, output_file)
    print("Done!")

if __name__ == '__main__':
    main()
