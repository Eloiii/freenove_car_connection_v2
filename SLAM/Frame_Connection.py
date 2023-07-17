import numpy as np


class Connection:

    def __init__(self, frame_id1, frame_id2, relative_pose, matches):
        self.frame_id1 = frame_id1
        self.frame_id2 = frame_id2
        self.relative_pose = relative_pose
        self.matches = np.array(matches)

    def get_frame_ids(self):
        return np.array([self.frame_id1, self.frame_id2])
