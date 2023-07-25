import os

import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np


def draw_kp(img):
    orb = cv.ORB_create()

    kp, des = orb.detectAndCompute(img, None)
    kp_img = cv.drawKeypoints(img, kp, None, color=(0, 255, 0), flags=0)

    return kp_img


def generate_kp_images():
    print(f'{n_files} to be analysed')

    orb_dir = 'orb_' + images_dir
    os.mkdir(f'./{orb_dir}')
    for k in range(n_files):
        print(f'image {k}.jpg')
        img = cv.imread(f'../{images_dir}/{k}.jpg', cv.IMREAD_GRAYSCALE)

        kp_img = draw_kp(img)

        cv.imwrite(f'./{orb_dir}/{k}.jpg', kp_img)


def find_matching():
    print(f'{n_files} to be analysed')
    orb = cv.ORB_create()
    orb_dir = 'matching_' + images_dir
    os.mkdir(f'./{orb_dir}')

    for k in range(n_files - 2):
        print(f'images {k}.jpg and {k + 1}.jpg')
        img1 = cv.imread(f'../{images_dir}/{k}.jpg', cv.IMREAD_GRAYSCALE)
        img2 = cv.imread(f'../{images_dir}/{k + 1}.jpg', cv.IMREAD_GRAYSCALE)

        kp1, des1 = orb.detectAndCompute(img1, None)
        kp2, des2 = orb.detectAndCompute(img2, None)

        bf = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)
        matches = bf.match(des1, des2)
        matches = sorted(matches, key=lambda x: x.distance)

        img3 = cv.drawMatches(img1, kp1, img2, kp2, matches[:10], None,
                              flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        cv.imwrite(f'./{orb_dir}/{k}-{k + 1}.jpg', img3)


def extractRt(F):
    W = np.mat([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=float)
    U, d, Vt = np.linalg.svd(F)
    # assert np.linalg.det(U) > 0
    if np.linalg.det(Vt) < 0:
        Vt *= -1.0
    R = np.dot(np.dot(U, W), Vt)
    if np.sum(R.diagonal()) < 0:
        R = np.dot(np.dot(U, W.T), Vt)
    t = U[:, 2]
    ret = np.eye(4)
    ret[:3, :3] = R
    ret[:3, 3] = t
    # print(d)
    return ret


def triangulate(pose1, pose2, pts1, pts2):
    ret = np.zeros((pts1.shape[0], 4))
    pose1 = np.linalg.inv(pose1)
    pose2 = np.linalg.inv(pose2)
    for i, p in enumerate(zip(pts1, pts2)):
        A = np.zeros((4, 4))
        A[0] = p[0][0] * pose1[2] - pose1[0]
        A[1] = p[0][1] * pose1[2] - pose1[1]
        A[2] = p[1][0] * pose2[2] - pose2[0]
        A[3] = p[1][1] * pose2[2] - pose2[1]
        _, _, vt = np.linalg.svd(A)
        ret[i] = vt[3]
    return ret


def draw_points(points, img, name):
    image = img
    for point in points:
        center = (int(point[0]), int(point[1]))
        image = cv.circle(image, center, radius=1, color=(0, 255, 255), thickness=3)

    cv.imwrite(name, image)


K = np.matrix(
    [[692.1783869, 0., 353.07150929], [0., 704.4016042, 178.98211959], [0., 0., 1.]]).A
dist = np.array([[0.12111298, 0.26876559, -0.04746641, -0.00175447, -1.04781593]])

K2 = np.matrix([[610.74086543, 0., 288.41585705], [0., 609.96728207, 254.08996828], [0., 0., 1.]])
dist2 = np.array([[0.1870135, -0.44984605, 0.0081328, -0.01649483, -2.30811376]])

K3 = np.matrix([[590.74297264, 0., 324.63603709], [0., 585.79496849, 142.70731962], [0., 0., 1.]])
dist3 = np.array([[-0.04404331, 0.46255196, -0.01871533, 0.01093527, -0.90253976]])


def undistort(x, y, k, distortion, iter_num=3):
    k1, k2, p1, p2, k3 = distortion[0]
    fx, fy = k[0, 0], k[1, 1]
    cx, cy = k[:2, 2]
    x = (x - cx) / fx
    x0 = x
    y = (y - cy) / fy
    y0 = y
    for _ in range(iter_num):
        r2 = x ** 2 + y ** 2
        k_inv = 1 / (1 + k1 * r2 + k2 * r2 ** 2 + k3 * r2 ** 3)
        delta_x = 2 * p1 * x * y + p2 * (r2 + 2 * x ** 2)
        delta_y = p1 * (r2 + 2 * y ** 2) + 2 * p2 * x * y
        x = (x0 - delta_x) * k_inv
        y = (y0 - delta_y) * k_inv
    return np.array((x * fx + cx, y * fy + cy))


def slam(n_images):
    orb = cv.ORB_create()
    bf = cv.BFMatcher(cv.NORM_HAMMING)

    starting_image = 75

    img1 = cv.imread(f'../{images_dir}/{starting_image}.jpg', cv.IMREAD_GRAYSCALE)

    pre_points, pre_features = orb.detectAndCompute(img1, None)

    xpoints = np.array([])
    ypoints = np.array([])
    zpoints = np.array([])

    for k in range(starting_image + 1, starting_image + n_images + 1):
        curr_frame = cv.imread(f'../{images_dir}/{k}.jpg', cv.IMREAD_GRAYSCALE)

        curr_points, curr_features = orb.detectAndCompute(curr_frame, None)

        matches = bf.knnMatch(pre_features, curr_features, k=2)

        pre_matched_points = []
        curr_matched_points = []
        for i, (m, n) in enumerate(matches):
            if m.distance < 0.8 * n.distance:
                curr_matched_points.append(curr_points[m.trainIdx].pt)
                pre_matched_points.append(pre_points[m.queryIdx].pt)

        pre_matched_points = np.array(pre_matched_points)
        curr_matched_points = np.array(curr_matched_points)

        F, mask = cv.findFundamentalMat(pre_matched_points, curr_matched_points, method=cv.RANSAC)

        inlier_pre_points = pre_matched_points[mask.ravel() == 1]
        inlier_curr_points = curr_matched_points[mask.ravel() == 1]

        draw_points(inlier_curr_points, curr_frame, 'inlierpoints.jpg')

        # pre_pts_norm = cv.undistortPoints(inlier_pre_points, K, distCoeffs=dist)
        # curr_pts_norm = cv.undistortPoints(inlier_curr_points, K, distCoeffs=dist)

        pre_pts_norm = []
        curr_pts_norm = []

        for point in inlier_pre_points:
            pre_pts_norm.append(undistort(point[0], point[1], K, dist))
        for point in inlier_curr_points:
            curr_pts_norm.append(undistort(point[0], point[1], K, dist))

        pre_pts_norm = np.array(pre_pts_norm)
        curr_pts_norm = np.array(curr_pts_norm)

        # image = cv.undistort(curr_frame, K, dist, None, K)
        # for point in curr_pts_norm:
        #     center = (int(point[0]), int(point[1]))
        #     image = cv.circle(image, center, radius=1, color=(0, 255, 255), thickness=2)
        #
        # cv.imwrite('normpointsonnormimage.jpg', image)

        E, _ = cv.findEssentialMat(pre_pts_norm, curr_pts_norm, K, method=cv.RANSAC, prob=0.999, threshold=3.0)

        _, R, t, _ = cv.recoverPose(E, pre_pts_norm, curr_pts_norm)

        # img_undistort = cv.undistort(curr_frame, K, dist, None, K)
        # cv.imwrite('undistort.jpg', img_undistort)
        # cv.imwrite('base.jpg', curr_frame)

        # P1 = np.hstack((K, np.zeros((3, 1))))
        # P2 = np.hstack((np.dot(K, R), np.dot(K, t)))
        #
        # points_4d_homogeneous = cv.triangulatePoints(P1, P2, pre_pts_norm, curr_pts_norm)
        #
        # points_3d = cv.convertPointsFromHomogeneous(points_4d_homogeneous.T)

        # camera_location = -np.dot(R.T, t)

        M_r = np.hstack((R, t))
        M_l = np.hstack((np.eye(3, 3), np.zeros((3, 1))))

        P_l = np.dot(K, M_l)
        P_r = np.dot(K, M_r)
        point_4d_hom = cv.triangulatePoints(P_l, P_r, np.expand_dims(inlier_pre_points, axis=1),
                                            np.expand_dims(inlier_curr_points, axis=1))
        point_4d = point_4d_hom / np.tile(point_4d_hom[-1, :], (4, 1))
        points_3d = point_4d[:3, :].T

        # points_3d = cv.convertPointsFromHomogeneous(point_4d_hom.T)

        # xpoints = np.append(xpoints, points_3d[:, 0, 0])
        # ypoints = np.append(ypoints, points_3d[:, 0, 1])
        # zpoints = np.append(zpoints, points_3d[:, 0, 2])

        xpoints = np.append(xpoints, points_3d[:, 0])
        ypoints = np.append(ypoints, points_3d[:, 1])
        zpoints = np.append(zpoints, points_3d[:, 2])

        pre_points, pre_features = curr_points, curr_features

    return xpoints, ypoints, zpoints


def slamv2(n_images):
    def estimate_pose(prev_image, current_image):
        # Create an ORB object
        orb = cv.ORB_create()

        # Find the descriptors and keypoints for the previous and current images
        prev_keypoints, prev_descriptors = orb.detectAndCompute(prev_image, None)
        current_keypoints, current_descriptors = orb.detectAndCompute(current_image, None)

        # Create a matcher object
        matcher = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)

        # Match the descriptors
        matches = matcher.match(prev_descriptors, current_descriptors)

        # Sort the matches by distance
        matches = sorted(matches, key=lambda x: x.distance)

        # Select only the top matches
        num_matches = int(len(matches) * 0.4)
        matches = matches[:num_matches]

        # Extract matched keypoints
        prev_matched_pts = np.float32([prev_keypoints[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
        current_matched_pts = np.float32([current_keypoints[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

        # Estimate the essential matrix
        essential_matrix, mask = cv.findEssentialMat(prev_matched_pts, current_matched_pts)

        # Recover the relative camera pose
        _, R, t, _ = cv.recoverPose(essential_matrix, prev_matched_pts, current_matched_pts)

        return R, t, prev_matched_pts, current_matched_pts

    def triangulate_points(camera_matrix, current_pose, prev_points, current_points):

        # Normalize the image coordinates
        prev_points_norm = cv.undistortPoints(prev_points, camera_matrix, None)
        current_points_norm = cv.undistortPoints(current_points, camera_matrix, None)

        # Construct the projection matrices
        P1 = np.hstack((np.eye(3), np.zeros((3, 1))))
        P2 = np.hstack((current_pose[0], current_pose[1]))

        # Triangulate the points
        points_4d_homogeneous = cv.triangulatePoints(P1, P2, prev_points_norm, current_points_norm)

        # Convert homogeneous coordinates to 3D points
        points_3d = cv.convertPointsFromHomogeneous(points_4d_homogeneous.T)

        return points_3d

    starting_frame = 36

    pointss = []

    previous_image = cv.imread(f'../{images_dir}/{starting_frame}.jpg', cv.IMREAD_GRAYSCALE)

    for k in range(starting_frame + 1, n_images + starting_frame + 1):
        current_image = cv.imread(f'../{images_dir}/{k}.jpg', cv.IMREAD_GRAYSCALE)

        R, t, prev_points, curr_points = estimate_pose(previous_image, current_image)

        points_3d = triangulate_points(K, (R, t), prev_points, curr_points)

        for j in range(points_3d.shape[0]):
            coordinates = points_3d[j, 0, :]
            pointss.append(coordinates)

        previous_image = current_image

    return np.array(pointss)


if __name__ == '__main__':
    dirs = os.listdir('..')
    images_dir = list(filter(lambda directory: directory.startswith('images_2023-06-21_16:00:55.626868'), dirs))[0]
    n_files = len(os.listdir(f'../{images_dir}'))

    # draw_kp()

    find_matching()

    # generate_kp_images()

    # xpoints, ypoints, zpoints = slam(1)

    # points = slamv2(1)
    # print(points)
