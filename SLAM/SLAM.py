import os

import cv2 as cv
import numpy as np


# turn [[x,y]] -> [[x,y,1]]
def add_ones(x):
    if len(x.shape) == 1:
        return add_ones_1D(x)
    else:
        return np.concatenate([x, np.ones((x.shape[0], 1))], axis=1)


# turn [[x,y]] -> [[x,y,1]]
def add_ones_1D(x):
    # return np.concatenate([x,np.array([1.0])], axis=0)
    return np.array([x[0], x[1], 1])
    # return np.append(x, 1)


class Frame:
    def __init__(self, idx, kp, desc, kpu, kpn):
        self.index = idx
        self.kp = kp
        self.desc = desc
        self.kpu = kpu
        self.kpn = kpn
        self.pose = np.eye(4)

    def set_pose(self, pose):
        self.pose = pose


class SLAM:
    def __init__(self):
        self.state = 'NO_FRAME'
        self.orb = cv.ORB_create(nfeatures=2000, scaleFactor=1.2, nlevels=8)
        self.bf = cv.BFMatcher(cv.NORM_HAMMING)
        self.frames = []
        self.curr_frame_idx = 0
        self.ref_frame = None
        self.points = []

        self.K = np.matrix([[590.74297264, 0., 324.63603709], [0., 585.79496849, 142.70731962], [0., 0., 1.]])
        self.Kinv = np.linalg.inv(self.K)
        self.dist = np.array([[-0.04404331, 0.46255196, -0.01871533, 0.01093527, -0.90253976]])

        dirs = os.listdir('..')
        self.images_dir = list(filter(lambda directory: directory.startswith('images_2023-06-27'), dirs))[0]
        self.n_files = len(os.listdir(f'../{self.images_dir}'))

        self.video = '/home/eloi/stage4A/freenove_car_connection_v2/SLAM/calibration_images_4/VID20230627162017.mp4'

    def run(self):
        for k in range(self.n_files):
            img = cv.imread(f'../{self.images_dir}/{k}.jpg', cv.IMREAD_GRAYSCALE)
            self.iter(img, k)

    def run_video(self):
        vidcap = cv.VideoCapture(self.video)
        success, image = vidcap.read()
        idx = 0
        while success and idx < 50:
            image = cv.cvtColor(image, cv.COLOR_RGB2GRAY)
            self.iter(image, idx)
            success, image = vidcap.read()
            idx += 1

        print(self.points)

    def unproject_points(self, uvs):
        return np.array(np.dot(self.Kinv, add_ones(uvs).T).T[:, 0:2])

    def run_orb(self, img, curr_frame_idx):
        kp, desc = self.orb.detectAndCompute(img, None)
        kp = np.array([p.pt for p in kp])
        kpu = cv.undistortPoints(kp, self.K, self.dist, None, self.K)
        kpu = kpu.reshape(kpu.shape[0], 2)
        kpn = self.unproject_points(kpu)
        frame = Frame(curr_frame_idx, kp, desc, kpu, kpn)
        self.frames.append(frame)
        return frame

    def match_frames(self, frame1, frame2):
        matches = self.bf.knnMatch(frame1.desc, frame2.desc, k=2)

        frame1_match_idx = []
        frame2_match_idx = []
        dist_match = [None] * max(len(frame2.desc), len(frame2.desc))
        index_match = dict()
        for i, (m, n) in enumerate(matches):
            if m.distance < 0.8 * n.distance:
                dist = dist_match[m.trainIdx]
                if dist is None:
                    dist_match[m.trainIdx] = m.distance
                    frame1_match_idx.append(m.queryIdx)
                    frame2_match_idx.append(m.trainIdx)
                    index_match[m.trainIdx] = len(frame2_match_idx) - 1
                else:
                    if m.distance < dist:
                        index = index_match[m.trainIdx]
                        frame1_match_idx[index] = m.queryIdx
                        frame2_match_idx[index] = m.trainIdx

        return np.array(frame1_match_idx), np.array(frame2_match_idx)

    def estimate_pose(self, ref_points, curr_points):
        E, mask_E = cv.findEssentialMat(curr_points, ref_points, self.K, cv.RANSAC, 0.999, threshold=0.0003)
        _, R, t, _ = cv.recoverPose(E, curr_points, ref_points)

        ret = np.eye(4)
        ret[:3, :3] = R
        ret[:3, 3] = t.T
        return ret, mask_E

    def triangulate_normalised_points(self, curr_frame, ref_frame, cur_pts, ref_pts):
        P1w = curr_frame.pose[:3, :]  # [R1w, t1w]
        P2w = ref_frame.pose[:3, :]  # [R2w, t2w]

        point_4d_hom = cv.triangulatePoints(P1w, P2w, cur_pts.T, ref_pts.T)
        good_pts_mask = np.where(point_4d_hom[3] != 0)[0]
        point_4d = point_4d_hom / point_4d_hom[3]

        points_3d = point_4d[:3, :].T
        return points_3d, good_pts_mask

    def project_points(self, points_3d, proj_matrix):
        points_homogeneous = np.hstack((points_3d, np.ones((points_3d.shape[0], 1))))
        points_2d_homogeneous = np.dot(proj_matrix, points_homogeneous.T)
        points_2d_homogeneous /= points_2d_homogeneous[2, :]

        points_2d = points_2d_homogeneous[:2, :].T

        return points_2d

    def filter_triangulated_points(self, points_3d):
        min_distance = 1.0
        max_distance = 500.0
        reprojection_threshold = 2.0

        # Distance-based filtering
        distances = np.linalg.norm(points_3d, axis=1)
        good_points_mask = np.logical_and(distances > min_distance, distances < max_distance)
        filtered_points_dist = points_3d[good_points_mask]

        # Reprojection error filtering (assuming camera poses and intrinsic parameters are available)

        projection_matrix = np.array(self.K @ np.hstack((np.eye(3), np.zeros((3, 1)))))

        # Example 2D points corresponding to the triangulated points (numpy array of shape (N, 2))
        image_points = self.project_points(points_3d, projection_matrix)

        projected_points = np.dot(projection_matrix,
                                  np.concatenate(
                                      (filtered_points_dist, np.ones((image_points[good_points_mask].shape[0], 1))),
                                      axis=1).T)
        projected_points = projected_points[:2, :] / projected_points[2, :]
        reprojection_errors = np.linalg.norm(projected_points.T - image_points[good_points_mask], axis=1)

        good_points_mask[good_points_mask] &= (reprojection_errors < reprojection_threshold)
        filtered_points_reproj = points_3d[good_points_mask]
        return filtered_points_reproj

    def iter(self, img, curr_frame_idx):
        if self.state == 'NO_FRAME':
            frame = self.run_orb(img, curr_frame_idx)
            self.ref_frame = frame
            self.state = 'NOT_INIT'

        elif self.state == 'NOT_INIT':
            self.run_orb(img, curr_frame_idx)

            curr_frame = self.frames[curr_frame_idx]

            # if curr_frame.index - self.ref_frame.index > 5:
            self.ref_frame = self.frames[-2]

            ref_frame = self.ref_frame
            print(f'Comparing frame {curr_frame.index} and {ref_frame.index}')

            match_idx_curr, match_idx_ref = self.match_frames(curr_frame, ref_frame)

            Trc, mask_E = self.estimate_pose(ref_frame.kpn[match_idx_ref], curr_frame.kpn[match_idx_curr])

            # print(Trc)

            # invT
            ret = np.eye(4)
            R_T = Trc[:3, :3].T
            t = Trc[:3, 3]
            ret[:3, :3] = R_T
            ret[:3, 3] = -R_T @ t

            ref_frame.set_pose(np.eye(4))  # nothing change
            curr_frame.set_pose(ret)

            idx_curr_inliers = match_idx_curr[mask_E.ravel() == 1]
            idx_ref_inliers = match_idx_ref[mask_E.ravel() == 1]

            pts3d, mask_pts3d = self.triangulate_normalised_points(curr_frame, ref_frame,
                                                                   curr_frame.kpn[idx_curr_inliers],
                                                                   ref_frame.kpn[idx_ref_inliers])

            filtered_3d_points = self.filter_triangulated_points(pts3d)

            if len(filtered_3d_points) > 50:
                # self.state = 'INITIALISED'
                self.points.append(filtered_3d_points)
                print(f'Initialised with {len(filtered_3d_points)} points')

        elif self.state == 'INITIALISED':
            self.run_orb(img, curr_frame_idx)

            curr_frame = self.frames[curr_frame_idx]
            ref_frame = self.ref_frame[-2]

            pass


if __name__ == '__main__':
    slam = SLAM()
    slam.run_video()
