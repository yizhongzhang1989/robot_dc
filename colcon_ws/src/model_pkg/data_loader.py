import os, glob, json
from typing import List, Tuple
import numpy as np
from PIL import Image
from torch.utils.data import Dataset
import torchvision.transforms as T
from scipy.spatial.transform import Rotation as R
from utils import project_pose


class RobotDataset(Dataset):
	def __init__(self, data_root, target_size: Tuple[int,int]=(448,448), grid_size: int=32, is_training: bool=True):
		self.root, self.target_size, self.grid_size = data_root, target_size, grid_size
		self.transform = T.Compose([T.Resize(self.target_size), T.ToTensor(), T.Normalize([0.485,0.456,0.406],[0.229,0.224,0.225])])
		self.k = np.array([
			[1583.713421584196, 0.0, 982.8706765419257],
			[0.0, 1583.7034747152047, 543.7404915500364],
			[0.0, 0.0, 1.0
		]])

		# Add robot_pose matrix
		quat = [0.6925515674653875, 0.7153857677058679, -0.049108613211706946, 0.07863761106204165]
		position = [-0.05048721377287727, -0.42946610406790175, 0.7610675973184527]
		rotation_matrix = R.from_quat(quat).as_matrix()
		self.robot_pose = np.eye(4)
		self.robot_pose[:3, :3] = rotation_matrix
		self.robot_pose[:3, 3] = position
		
		self.cam2end = np.array([
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

		self.index: List[Tuple] = []
		for cmd in glob.glob(os.path.join(self.root, '**', 'data.json'), recursive=True):
			ep = os.path.dirname(cmd)
			if os.path.exists(os.path.join(ep, "image.png")):
				self.index.append(ep)

	def __len__(self): return len(self.index)

	def __getitem__(self, idx: int):
		ep = self.index[idx]
		img = Image.open(os.path.join(ep,'image.png')).convert('RGB')
		json_data = json.load(open(os.path.join(ep,'data.json'),'r'))
		
		# Get original image dimensions
		original_width, original_height = img.size
		
		# Project poses to pixel coordinates
		origin_pixel = project_pose(json_data['origin_pose'], self.k, self.cam2end, self.robot_pose)
		action_pixel = project_pose(json_data['action_pose'], self.k, self.cam2end, self.robot_pose)
		
		# Scale pixels from original size to target size (448x448)
		scale_x = self.target_size[0] / original_width
		scale_y = self.target_size[1] / original_height
		
		scaled_origin_pixel = (origin_pixel[0] * scale_x, origin_pixel[1] * scale_y)
		scaled_action_pixel = (action_pixel[0] * scale_x, action_pixel[1] * scale_y)
		
		# Convert scaled pixels to grid indices
		def pixels_to_grid_index(scaled_pixel, grid_size, target_size):
			grid_x = int(scaled_pixel[0] * grid_size / target_size[0])
			grid_y = int(scaled_pixel[1] * grid_size / target_size[1])
			grid_x = max(0, min(grid_x, grid_size - 1))
			grid_y = max(0, min(grid_y, grid_size - 1))
			return grid_y * grid_size + grid_x
		
		origin_grid = pixels_to_grid_index(scaled_origin_pixel, self.grid_size, self.target_size)
		action_grid = pixels_to_grid_index(scaled_action_pixel, self.grid_size, self.target_size)
		
		return {'img':self.transform(img), 'origin_grid':origin_grid, 'action_grid':action_grid, 'rgb_img': np.array(img)}