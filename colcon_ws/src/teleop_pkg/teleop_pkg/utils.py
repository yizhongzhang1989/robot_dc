import os
import json
import cv2


class DataSaver:
    def __init__(self, save_root="saved_data"):
        self.save_root = save_root
        os.makedirs(self.save_root, exist_ok=True)
        self.index = self._get_initial_index()

    def _get_initial_index(self):
        existing = [int(name) for name in os.listdir(self.save_root) if name.isdigit()]
        return max(existing) + 1 if existing else 0

    def save(self, image, origin_pose, action_pose):
        save_dir = os.path.join(self.save_root, str(self.index))
        os.makedirs(save_dir, exist_ok=True)

        # Save image
        image_path = os.path.join(save_dir, "image.png")
        cv2.imwrite(image_path, image)
        print(origin_pose)
        # Save pose and point data
        org = [origin_pose.pose.position.x, origin_pose.pose.position.y, 
               origin_pose.pose.position.z]
        act = [action_pose.pose.position.x, action_pose.pose.position.y, 
               action_pose.pose.position.z]
        data = {
            "origin_pose": org,
            "action_pose": act,
        }
        json_path = os.path.join(save_dir, "data.json")
        with open(json_path, "w") as f:
            json.dump(data, f, indent=4)

        self.index += 1

    def pose_to_position_dict(self, pose_msg):
        return {
            "x": pose_msg.position.x,
            "y": pose_msg.position.y,
            "z": pose_msg.position.z
        }