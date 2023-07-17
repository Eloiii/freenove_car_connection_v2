import cv2 as cv
import numpy as np
from Frame_Connection import Connection


def get_most_common_value(array):
    flattened = array.flatten()
    if -1 in flattened:
        flattened = flattened[flattened != -1]
    most_common = np.argmax(np.bincount(flattened))
    return most_common


class MapPoint:
    def __init__(self, pt3d, pt2d):
        self.pt_3d = pt3d
        self.pt_2d = pt2d
        self.kf_observations = dict()
        self.f_observations = dict()

    def add_observation(self, fidx, is_kf, idx):
        if is_kf:
            self.kf_observations[fidx] = idx
        self.f_observations[fidx] = idx


class Map:
    def __init__(self):
        self.points = []
        # self.points_3d = np.empty((0, 3))
        # self.points_2d = []
        self.keyframes = []
        self.frames = []
        self.keyframes_matches = {}
        self.connections = []

    def add_points(self, points_3d, Trc, K, dist, frame_idx, is_kf):
        curr_points_len = len(self.points)

        R = Trc[:3, :3]
        t = Trc[:3, 3].T

        points_2d, _ = cv.projectPoints(points_3d, R, t, K, dist)
        points_2d = points_2d.reshape((points_2d.shape[0], 2))

        for k, p in enumerate(points_3d):
            mp = MapPoint(p, points_2d[k])
            for k_f, frame_id in enumerate(frame_idx):
                frame = self.get_f_with_id(frame_id)
                mp.add_observation(frame_id, is_kf[k_f], frame.points_filtered_idx[k])
            self.points.append(mp)

        return np.arange(curr_points_len, len(self.points), 1, dtype=int)

    def add_keyframe(self, frame):
        frame.is_keyframe = True
        self.keyframes.append(frame)

    def add_connection(self, frame1_id, frame2_id, relative_pose=None, matches=None):
        connection = Connection(frame1_id, frame2_id, relative_pose, matches)
        self.connections.append(connection)

    def get_connected_frames(self, frame_idx):
        res = []
        for connection in self.connections:
            conn_ids = connection.get_frame_ids()
            if frame_idx in conn_ids:
                res.append(conn_ids[conn_ids != frame_idx][0])

        return res

    def get_connection(self, id1, id2):
        for connection in self.connections:
            conn_ids = connection.get_frame_ids()
            if id1 in conn_ids and id2 in conn_ids:
                return connection
        return None

    def get_last_keyframe_idx(self):
        return self.keyframes[-1].id

    def get_f_with_id(self, kf_id):
        return [frame for frame in self.frames if frame.id == kf_id][0]

    def find_views_of_world_point(self, points_idx):
        res = []
        for p in points_idx:
            tmp = []
            for view in self.keyframes:
                if len(view.points_idx_in_world) == 0:
                    continue
                if p in view.points_idx_in_world:
                    tmp.append(view.id)
            res.append(tmp)
        max_length = max(len(subarray) for subarray in res)

        padded_data = [subarray + [-1] * (max_length - len(subarray)) for subarray in res]

        data_array = np.array(padded_data)

        return data_array

    def get_desc(self, new_map_points_idx):
        views_of_points = self.find_views_of_world_point(new_map_points_idx)
        desc = []
        for i, point in enumerate(new_map_points_idx):
            for frame_idx in views_of_points[i]:
                res = self.frames[frame_idx].get_correspondence(point)
                if res is not None:
                    desc.append(res)
                    break
        return np.array(desc)
