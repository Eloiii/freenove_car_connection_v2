import numpy as np


class Frame:
    def __init__(self, idx, real_kp, kp, desc, kpu, kpn, image):
        self.id = idx
        self.image = image
        self.real_kp = np.array(real_kp)
        self.kp = kp
        self.desc = desc
        self.kpu = kpu
        self.kpn = kpn
        self.pose = np.eye(4)
        self.is_keyframe = False
        self.points_idx_in_world = np.array([], dtype=int)
        self.points_filtered_idx = None
        self.desc_filtered_idx = None
        self.desc_world_point = dict()

    def add_correspondence(self, world_points_idx, desc):
        assert len(world_points_idx) == len(desc)
        for i, point in enumerate(world_points_idx):
            self.desc_world_point[point] = desc[i]

    def get_correspondence(self, point_idx):
        if point_idx not in self.desc_world_point:
            return None
        if isinstance(self.desc_world_point[point_idx], np.ndarray):
            return self.desc_world_point[point_idx][0]
        return self.desc_world_point[point_idx]

    def set_pose(self, pose):
        self.pose = pose

    def add_points(self, points):
        self.points_idx_in_world = np.unique(np.append(self.points_idx_in_world, points))

    def add_filtered_desc(self, desc):
        for d in desc:
            self.desc_filtered_idx = np.append(self.desc_filtered_idx, d)
