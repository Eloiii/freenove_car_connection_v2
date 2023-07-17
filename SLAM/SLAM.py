import os

import cv2 as cv
import numpy as np
from Frame import Frame
from Map import Map
from good_phone_3d_points import get_points, get_poses
from DebugUtilities import *
from colorist import red, blue, yellow
from test_g20 import bundle_adjustment


def compute_F(intrinsics, pose1, pose2):
    R1 = pose1[:3, :3]
    t1 = pose1[:3, 3]

    R2 = pose2[:3, :3]
    t2 = pose2[:3, 3]

    R12 = np.transpose(R1) @ R2
    t12 = np.transpose(R1) @ (t2 - t1)

    t12x = np.array([[0, -t12[2], t12[1]],
                     [t12[2], 0, -t12[0]],
                     [-t12[1], t12[0], 0]])

    K_inv = np.linalg.inv(intrinsics)

    F = K_inv.T @ t12x @ R12 @ K_inv

    return F


def get_Rt(pose):
    R = pose[:3, :3]
    t = pose[:3, 3].T
    return R, t


# turn [[x,y]] -> [[x,y,1]]
def add_ones(x):
    if len(x.shape) == 1:
        return add_ones_1D(x)
    else:
        return np.concatenate([x, np.ones((x.shape[0], 1))], axis=1)


# turn [[x,y]] -> [[x,y,1]]
def add_ones_1D(x):
    return np.array([x[0], x[1], 1])


def get_most_common_value(array):
    flattened = array.flatten()
    if -1 in flattened:
        flattened = flattened[flattened != -1]
    most_common = np.argmax(np.bincount(flattened))
    return most_common


class SLAM:
    def __init__(self):
        self.state = 'NO_FRAME'
        self.orb = cv.ORB_create(nfeatures=2000, scaleFactor=1.2, nlevels=8)
        self.bf = cv.BFMatcher(cv.NORM_HAMMING)
        self.curr_frame_idx = 0
        self.ref_frame = None
        self.is_last_frame_kf = False

        self.map = Map()

        self.num_points_ref_kf = 0
        self.local_kf_idx_internal = []
        self.local_points_idx = []
        self.local_kf_idx = []

        self.K = np.matrix([[590.74297264, 0., 324.63603709], [0., 585.79496849, 142.70731962], [0., 0., 1.]])
        self.Kinv = np.array(np.linalg.inv(self.K))
        self.dist = np.array([[-0.04404331, 0.46255196, -0.01871533, 0.01093527, -0.90253976]])

        dirs = os.listdir('..')
        self.images_dir = list(filter(lambda directory: directory.startswith('images_2023-06-27'), dirs))[0]
        self.n_files = len(os.listdir(f'../{self.images_dir}'))

        self.video = '/home/eloi/stage4A/freenove_car_connection_v2/SLAM/calibration_images_4/VID20230627162017.mp4'

    def run(self):
        for k in range(self.n_files):
            img = cv.imread(f'../{self.images_dir}/{k}.jpg', cv.IMREAD_GRAYSCALE)
            self.iter(img, k)

    def run_video(self, process_n):
        vidcap = cv.VideoCapture(self.video)
        success, image = vidcap.read()
        idx = 0
        process_n = process_n if process_n > 0 else float('inf')
        while success and idx < process_n:
            image = cv.cvtColor(image, cv.COLOR_RGB2GRAY)
            self.iter(image, idx)
            success, image = vidcap.read()
            idx += 1
        points = self.map.points_3d

        cv.imwrite(f'{idx}.jpg', image)

        # self.display_plot()
        # points = get_points(3500)
        # points = get_poses()
        draw_3d(points[:, 0], points[:, 1], points[:, 2])
        # print(np.array(self.points))

    def unproject_points(self, uvs):
        return np.array(np.dot(self.Kinv, add_ones(uvs).T).T[:, 0:2])

    def run_orb(self, img, curr_frame_idx):
        kp, desc = self.orb.detectAndCompute(img, None)
        kp_pt = np.array([p.pt for p in kp])
        kpu = cv.undistortPoints(kp_pt, self.K, self.dist, None, self.K)
        kpu = kpu.reshape(kpu.shape[0], 2)
        kpn = self.unproject_points(kpu)
        frame = Frame(curr_frame_idx, kp, kp_pt, desc, kpu, kpn, img)
        self.map.frames.append(frame)
        return frame

    def match_frames(self, desc1, desc2):
        matches = self.bf.knnMatch(desc1, desc2, k=2)

        frame1_match_idx = []
        frame2_match_idx = []
        good_match = []
        dist_match = [None] * max(len(desc1), len(desc2))
        index_match = dict()
        for i, (m, n) in enumerate(matches):
            if m.distance < 0.8 * n.distance:
                dist = dist_match[m.trainIdx]
                good_match.append(m)
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
        E, mask_E = cv.findEssentialMat(curr_points, ref_points, focal=1.0, pp=(0., 0.),
                                        method=cv.RANSAC, prob=0.999, threshold=3.0)

        _, R, t, _ = cv.recoverPose(E, curr_points, ref_points)

        ret = np.eye(4)
        ret[:3, :3] = R
        ret[:3, 3] = t.T

        camera_location = -np.dot(R.T, t)
        return ret, mask_E, camera_location

    def triangulate_normalised_points(self, curr_frame, ref_frame, cur_pts, ref_pts):
        P1w = curr_frame.pose[:3, :]  # [R1w, t1w]
        P2w = ref_frame.pose[:3, :]  # [R2w, t2w]

        point_4d_hom = cv.triangulatePoints(P1w, P2w, cur_pts.T, ref_pts.T)
        good_pts_mask = np.where(point_4d_hom[3] != 0)[0]
        point_4d = point_4d_hom / point_4d_hom[3]

        points_3d = point_4d[:3, :].T
        return points_3d, good_pts_mask

    def filter_triangulated_points(self, points_3d, Trc, points_2d, curr_frame, ref_frame, scales1=None, scales2=None):
        # min_distance = 1.0
        # max_distance = 35.0
        # distances = np.linalg.norm(points_3d, axis=1)
        # mask_distance = np.logical_and(distances > min_distance, distances < max_distance)

        reprojection_threshold = np.sqrt(6) * np.minimum(scales1, scales2)

        R = Trc[:3, :3]
        t = Trc[:3, 3].T

        points_2d_reproj, _ = cv.projectPoints(points_3d, R, t, self.K, self.dist)
        reprojection_errors = np.linalg.norm(points_2d_reproj.squeeze() - points_2d, axis=1)
        mask_reproj_error = reprojection_errors <= reprojection_threshold
        mask_positive_z = points_3d[:, 2] > 0
        masks = mask_reproj_error & mask_positive_z
        filtered_points_reproj = points_3d[masks]

        min_parallax = 1

        ray1 = filtered_points_reproj - ref_frame.pose[:3, 3]
        ray2 = filtered_points_reproj - curr_frame.pose[:3, 3]
        cos_angles = np.sum(ray1 * ray2, axis=1) / (np.linalg.norm(ray1, axis=1) * np.linalg.norm(ray2, axis=1))

        is_valid = np.all(np.logical_and(cos_angles < np.cos(np.radians(min_parallax)), cos_angles > 0))

        return filtered_points_reproj, masks, is_valid

    def track_last_keyframe(self, curr_frame):

        last_keyframe = self.map.keyframes[-1]
        world_points_idx = last_keyframe.points_3d_idx_in_world

        # matches the features in the current frame  with the features in the last key frame
        # that have known corresponding world points.
        match_idx_curr, match_idx_kf = self.match_frames(curr_frame.desc,
                                                         last_keyframe.desc[last_keyframe.desc_filtered_idx])

        Trc, mask_E, camera_location = self.estimate_pose(last_keyframe.kpn[match_idx_kf],
                                                          curr_frame.kpn[match_idx_curr])

        ret = np.eye(4)
        R_T = Trc[:3, :3].T
        t = Trc[:3, 3]
        ret[:3, :3] = R_T
        ret[:3, 3] = -R_T @ t

        curr_frame.set_pose(ret)

        idx_curr_inliers = match_idx_curr[mask_E.ravel() == 1]
        idx_kf_inliers = match_idx_kf[mask_E.ravel() == 1]

        if (len(idx_curr_inliers)) <= 5:
            red(f'{len(idx_curr_inliers)} inliers')
            return

        # TODO refine camera pose

        return curr_frame.pose, world_points_idx[idx_kf_inliers], idx_curr_inliers

    def update_ref_kf_and_local_points(self, map_points_idx):
        frame_idx = self.map.find_views_of_world_point(map_points_idx)
        # ref_kf = with the most covisible map points
        ref_kf_idx = get_most_common_value(frame_idx)

        local_kfs = self.map.get_connected_frames(ref_kf_idx)
        local_kfs.append(ref_kf_idx)
        local_kfs_idx = np.array(local_kfs)

        point_idx = []
        for kf_id in local_kfs_idx:
            kf = self.map.get_kf_with_id(kf_id)
            point_idx.append(kf.points_3d_idx_in_world)
        max_length = max(len(sub) for sub in point_idx)
        same_size_point_idx = []
        for sub in point_idx:
            length = len(sub)
            if length < max_length:
                sub = np.concatenate((sub, [-1] * (max_length - length)))
            same_size_point_idx.append(sub)

        point_idx = np.array(same_size_point_idx)
        if len(point_idx.shape) == 3:
            point_idx = point_idx.reshape((point_idx.shape[0], point_idx.shape[2]))

        points_ref_kf = point_idx[local_kfs_idx == ref_kf_idx][0]

        num_points_ref_kf = len(points_ref_kf[points_ref_kf != -1])
        local_points_idx = point_idx.flatten()
        local_points_idx = local_points_idx[local_points_idx != -1]

        return local_points_idx, local_kfs_idx, num_points_ref_kf

    def remove_outliers_map_points(self, curr_pose, local_pts_idx):
        # curr_pose = self.map.keyframes[1].pose
        pts3d = self.map.points_3d[local_pts_idx]
        R, t = get_Rt(curr_pose)

        # points_2d_reproj, _ = cv.projectPoints(pts3d, R, t, self.K, self.dist)

        # filter in front of camera
        mask_positive_z = pts3d[:, 2] > 0
        # filtered_points_reproj = pts3d[mask_positive_z]

        min_parallax = 75

        camera_view_dir = R @ np.array([0, 0, 1])
        direction_to_points = pts3d - t.T

        dot_product = np.sum(camera_view_dir * direction_to_points, axis=1)
        norm_camera_view_dir = np.linalg.norm(camera_view_dir)
        norm_direction_to_points = np.linalg.norm(direction_to_points, axis=1)

        angles = np.arccos(dot_product / (norm_camera_view_dir * norm_direction_to_points))

        is_valid = angles < np.radians(min_parallax)

        inliers = is_valid & mask_positive_z
        return inliers

    def check_keyframe(self, num_skip_frames, num_points_kf, curr_frame_idx, map_points_idx, num_points_ref_kf):
        too_many_non_keyframe = curr_frame_idx > self.map.get_last_keyframe_idx() + num_skip_frames

        too_few_map_points = len(map_points_idx) < num_points_kf

        too_few_tracked_points = len(map_points_idx) < 0.9 * num_points_ref_kf

        return (too_many_non_keyframe or too_few_map_points) and too_few_tracked_points

    def track_local_map(self, map_points_idx, desc_idx, curr_pose, curr_desc, curr_pts, curr_frame_idx, num_skip_frames,
                        num_points_kf):

        if self.num_points_ref_kf == 0 or self.is_last_frame_kf:
            self.local_points_idx, self.local_kf_idx_internal, self.num_points_ref_kf = self.update_ref_kf_and_local_points(
                map_points_idx)

        """
        reference keyframe: keyframe with the most covisible map points
        local_points_idx: world maps points for the local keyframes (keyframes connected to the reference keyframe)
        local_kf_idx_internal: local keyframes indices
        num_points_ref_kf: number of points in the reference keyframe
        """

        new_map_points_idx = np.setdiff1d(self.local_points_idx, map_points_idx)
        local_desc = self.map.get_desc(new_map_points_idx)

        idx_inliers = self.remove_outliers_map_points(curr_pose, new_map_points_idx)

        if len(idx_inliers) == 0:
            red('No inliers while tracking local map')
            return False, [], []

        new_map_points_idx = new_map_points_idx[idx_inliers]
        local_desc = local_desc[idx_inliers]

        # TODO refine cam pose

        is_keyframe = self.check_keyframe(num_skip_frames, num_points_kf, curr_frame_idx, new_map_points_idx,
                                          self.num_points_ref_kf)

        self.local_kf_idx = self.local_kf_idx_internal

        return is_keyframe, new_map_points_idx, local_desc

    def add_ney_keyframe(self, curr_frame, new_points, desc, camera_pose):
        self.map.add_keyframe(curr_frame)
        curr_frame.add_points(new_points)

        for kf_id in self.local_kf_idx:
            keyframe = self.map.get_kf_with_id(kf_id)
            pts_3d_idx = keyframe.points_3d_idx_in_world
            desc_idx = keyframe.desc_filtered_idx
            _, ia, ib = np.intersect1d(pts_3d_idx, new_points, return_indices=True)

            pre_pose = keyframe.pose
            R_T = pre_pose[:3, :3]
            t_T = pre_pose[:3, 3]

            camera_pose_R = camera_pose[:3, :3]
            camera_pose_t = camera_pose[:3, 3]

            rel_pose_R = R_T.T @ camera_pose_R
            rel_pose_t = R_T.T @ (camera_pose_t - t_T)

            rel_pose = np.eye(4)
            rel_pose[:3, :3] = rel_pose_R
            rel_pose[:3, 3] = -rel_pose_R @ rel_pose_t

            if len(ia) > 5:
                self.map.add_connection(curr_frame.id, keyframe.id, rel_pose, [desc_idx[ia], desc[ib]])

        curr_frame.add_correspondence(new_points, desc)

    def create_new_map_points(self, curr_kf_pose):
        last_kf_id = self.map.get_last_keyframe_idx()

        connected_views_ids = self.map.get_connected_frames(last_kf_id)
        curr_kf = self.map.get_kf_with_id(last_kf_id)

        recent_point_idx = []
        for frame_id in connected_views_ids:
            kf = self.map.get_kf_with_id(frame_id)

            world_pts = self.map.points_3d[kf.points_3d_idx_in_world]
            world_pts_desc = self.map.get_desc(kf.points_3d_idx_in_world)
            kf_pose = kf.pose
            kf_pose_t = kf_pose[:3, 3]
            median_depth = np.median(np.linalg.norm(world_pts - kf_pose_t, axis=1))

            is_view_close = np.linalg.norm(kf_pose_t - curr_kf_pose[:3, 3]) / median_depth < 0.01

            if is_view_close:
                continue

            curr_kf_desc = self.map.get_desc(curr_kf.points_3d_idx_in_world)

            unmatched_idx1 = np.setdiff1d(np.arange(0, len(kf.desc)), world_pts_desc)
            unmatched_idx2 = np.setdiff1d(np.arange(0, len(curr_kf.desc)), curr_kf_desc)

            unmatched_desc1 = kf.desc[unmatched_idx1]
            unmatched_desc2 = curr_kf.desc[unmatched_idx2]

            match_idx_1, match_idx_2 = self.match_frames(unmatched_desc1, unmatched_desc2)

            if len(match_idx_1) == 0:
                continue

            matched_pts_1 = kf.kpu[match_idx_1]
            matched_pts_2 = curr_kf.kpu[match_idx_2]

            inliers1 = match_idx_1
            inliers2 = match_idx_2

            uScales2 = np.array([pt.octave + 1 for pt in kf.real_kp[match_idx_1]])

            F, mask = cv.findFundamentalMat(matched_pts_1, matched_pts_2, method=cv.RANSAC)
            Fbis = compute_F(self.K, kf_pose, curr_kf_pose)

            epiLines = cv.computeCorrespondEpilines(matched_pts_2.reshape(-1, 1, 2), 2, F)
            epiLines = epiLines.reshape(-1, 3)

            _, _, vt = np.linalg.svd(F)
            epipole_homogeneous = vt[-1]
            epipole = (epipole_homogeneous / epipole_homogeneous[2])[:2]

            dist_to_epipole = np.linalg.norm(matched_pts_2 - epipole, axis=1)

            dist_to_line = np.abs(
                np.sum(epiLines * np.hstack((matched_pts_1, np.ones((len(matched_pts_1), 1)))), axis=1))
            dist_to_line /= np.linalg.norm(epiLines[:, :2], axis=1)

            # Filter points based on distance thresholds
            epipole_mask = (dist_to_line < 2 * uScales2) & (dist_to_epipole > 10 * uScales2)

            # Filter matched points and index pairs
            matched_pts_1 = matched_pts_1[epipole_mask]
            matched_pts_2 = matched_pts_2[epipole_mask]

            inliers1 = inliers1[epipole_mask]
            inliers2 = inliers2[epipole_mask]

            min_parallax = 3

            ray1 = add_ones(matched_pts_1) - kf.pose[:3, 3]
            ray2 = add_ones(matched_pts_2) - curr_kf.pose[:3, 3]
            cos_angles = np.sum(ray1 * ray2, axis=1) / (np.linalg.norm(ray1, axis=1) * np.linalg.norm(ray2, axis=1))

            parallax_mask = np.logical_and(cos_angles < np.cos(np.radians(min_parallax)), cos_angles > 0)

            matched_pts_1 = matched_pts_1[parallax_mask]
            matched_pts_2 = matched_pts_2[parallax_mask]

            inliers1 = inliers1[parallax_mask]
            inliers2 = inliers2[parallax_mask]

            if len(inliers1) == 0:
                continue

            pts3d, mask_pts3d = self.triangulate_normalised_points(kf, curr_kf, kf.kpn[inliers1], curr_kf.kpn[inliers2])

            scales1 = np.array([pt.octave + 1 for pt in kf.real_kp[inliers1]])
            scales2 = np.array([pt.octave + 1 for pt in curr_kf.real_kp[inliers2]])

            filtered_3d_points, mask, is_valid = self.filter_triangulated_points(pts3d, kf_pose,
                                                                                 kf.kpu[inliers1],
                                                                                 kf, curr_kf, scales1, scales2)

            if len(filtered_3d_points) == 0:
                continue

            new_points_idx = self.map.add_points(filtered_3d_points, kf_pose, self.K, self.dist)

            yellow(f'added {len(new_points_idx)} points to the map')

            kf.add_points(new_points_idx)
            kf.add_filtered_desc(inliers1[mask])
            curr_kf.add_points(new_points_idx)
            curr_kf.add_filtered_desc(inliers2[mask])

            kf.add_correspondence(new_points_idx, inliers1[mask])
            curr_kf.add_correspondence(new_points_idx, inliers2[mask])

            existing_connection = self.map.get_connection(kf.id, curr_kf.id)
            old_matches = existing_connection.matches
            new_matches = np.hstack((old_matches, np.array([inliers1, inliers2])))
            existing_connection.matches = new_matches

    def iter(self, img, curr_frame_idx):
        if self.state == 'NO_FRAME':
            frame = self.run_orb(img, curr_frame_idx)
            self.ref_frame = frame
            self.state = 'NOT_INIT'

        elif self.state == 'NOT_INIT':
            self.run_orb(img, curr_frame_idx)

            curr_frame = self.map.frames[curr_frame_idx]

            if curr_frame.id - self.ref_frame.id > 5:
                self.ref_frame = self.map.frames[-2]

            ref_frame = self.ref_frame
            blue(f'Comparing frame {curr_frame.id} and {ref_frame.id}')

            match_idx_curr, match_idx_ref = self.match_frames(curr_frame.desc, ref_frame.desc)

            Trc, mask_E, camera_location = self.estimate_pose(ref_frame.kpn[match_idx_ref],
                                                              curr_frame.kpn[match_idx_curr])

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

            if (len(idx_curr_inliers)) <= 5:
                red(f'{len(idx_curr_inliers)} inliers')
                return

            pts3d, mask_pts3d = self.triangulate_normalised_points(curr_frame, ref_frame,
                                                                   curr_frame.kpn[idx_curr_inliers],
                                                                   ref_frame.kpn[idx_ref_inliers])

            scales1 = np.array([pt.octave + 1 for pt in curr_frame.real_kp[idx_curr_inliers]])
            scales2 = np.array([pt.octave + 1 for pt in ref_frame.real_kp[idx_ref_inliers]])

            filtered_3d_points, mask, is_valid = self.filter_triangulated_points(pts3d, Trc,
                                                                                 curr_frame.kpu[idx_curr_inliers],
                                                                                 curr_frame, ref_frame, scales1,
                                                                                 scales2)

            curr_frame.points_filtered_idx = np.nonzero(mask)[0]
            curr_frame.desc_filtered_idx = match_idx_curr[mask_E.ravel() == 1 & mask]
            ref_frame.points_filtered_idx = np.nonzero(mask)[0]
            ref_frame.desc_filtered_idx = match_idx_ref[mask_E.ravel() == 1 & mask]

            print(f'Filtered {len(filtered_3d_points)} {is_valid} points')

            if len(filtered_3d_points) > 50 and is_valid:
                self.state = 'INITIALISED'
                yellow(f'Initialised with {len(filtered_3d_points)} points')
                new_points_idx = self.map.add_points(filtered_3d_points, Trc, self.K, self.dist)

                self.map.add_keyframe(ref_frame)
                self.map.add_keyframe(curr_frame)
                ref_frame.add_points(new_points_idx)
                curr_frame.add_points(new_points_idx)

                self.map.add_connection(curr_frame.id, ref_frame.id, ret,
                                        [curr_frame.desc_filtered_idx, ref_frame.desc_filtered_idx])

                curr_frame.add_correspondence(new_points_idx, match_idx_curr[mask_E.ravel() == 1 & mask])
                ref_frame.add_correspondence(new_points_idx, match_idx_ref[mask_E.ravel() == 1 & mask])

                self.is_last_frame_kf = True
                # TODO refine and optimize points and camera pose (ceres, g2o...)
                bundle_adjustment(self.map.keyframes, self.map.points_3d)


        elif self.state == 'INITIALISED':
            self.run_orb(img, curr_frame_idx)

            curr_frame = self.map.frames[-1]

            camera_pose, world_points_idx, desc_idx = self.track_last_keyframe(curr_frame)

            """
            camera_pose: the updated camera pose for the current frame
            world_points_idx: the existing world points found in the new (current) frame
            desc_idx: features (descriptors) associated to the world_points found
            """

            blue(
                f'generated {len(world_points_idx)} points from frame {curr_frame_idx} with kf {self.map.get_last_keyframe_idx()}')

            num_skip_frames = 5
            num_points_kf = 100
            is_keyframe, new_points_idx, desc = self.track_local_map(world_points_idx, desc_idx, camera_pose,
                                                                     curr_frame.desc, curr_frame.kpn,
                                                                     curr_frame_idx, num_skip_frames, num_points_kf)

            curr_frame.points_filtered_idx = new_points_idx
            curr_frame.desc_filtered_idx = desc

            if not is_keyframe or len(new_points_idx) == 0:
                self.is_last_frame_kf = False
                return

            self.is_last_frame_kf = True
            yellow(f'new KF {curr_frame_idx}, with {len(new_points_idx)} points')

            self.add_ney_keyframe(curr_frame, new_points_idx, desc, camera_pose)

            self.create_new_map_points(camera_pose)


if __name__ == '__main__':
    slam = SLAM()
    slam.run_video(150)
