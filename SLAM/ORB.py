import os

import cv2 as cv
import numpy as np

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt


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


def slam():
    orb = cv.ORB_create()
    bf = cv.BFMatcher(cv.NORM_HAMMING)

    img1 = cv.imread(f'../{images_dir}/0.jpg', cv.IMREAD_GRAYSCALE)
    pre_points, pre_features = orb.detectAndCompute(img1, None)

    curr_frame_index = 1
    first_frame = img1

    map_initialised = False
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for k in range(100):
        curr_frame = cv.imread(f'../{images_dir}/{curr_frame_index}.jpg', cv.IMREAD_GRAYSCALE)
        curr_points, curr_features = orb.detectAndCompute(curr_frame, None)

        curr_frame_index += 1

        matches = bf.knnMatch(pre_features, curr_features, k=2)

        pre_matched_points = []
        curr_matched_points = []
        # ratio test as per Lowe's paper
        for i, (m, n) in enumerate(matches):
            if m.distance < 0.8 * n.distance:
                curr_matched_points.append(curr_points[m.trainIdx].pt)
                pre_matched_points.append(pre_points[m.queryIdx].pt)

        pre_matched_points = np.array(pre_matched_points)
        curr_matched_points = np.array(curr_matched_points)
        # tform, inlier_tform_idx = cv.findHomography(pre_matched_points, curr_matched_points, method=cv.RANSAC)

        F, mask = cv.findFundamentalMat(pre_matched_points, curr_matched_points, method=cv.FM_LMEDS)

        inlier_pre_points = pre_matched_points[mask.ravel() == 1]
        inlier_curr_points = curr_matched_points[mask.ravel() == 1]

        E, _ = cv.findEssentialMat(inlier_pre_points, inlier_curr_points)

        _, R, t, _ = cv.recoverPose(E, inlier_pre_points, inlier_curr_points)

        K = np.matrix(
            [[692.1783869, 0., 353.07150929], [0., 704.4016042, 178.98211959], [0., 0., 1.]]).A
        dist = np.array([[0.12111298, 0.26876559, -0.04746641, -0.00175447, -1.04781593]])

        pre_pts_norm = cv.undistortPoints(inlier_pre_points, K, dist)
        curr_pts_norm = cv.undistortPoints(inlier_curr_points, K, dist)

        P1 = np.hstack((K, np.zeros((3, 1))))
        P2 = np.hstack((np.dot(K, R), np.dot(K, t)))

        points_4d_homogeneous = cv.triangulatePoints(P1, P2, pre_pts_norm, curr_pts_norm)

        points_3d = cv.convertPointsFromHomogeneous(points_4d_homogeneous.T)

        camera_location = -np.dot(R.T, t)

        ax.scatter(points_3d[:, 0, 0], points_3d[:, 0, 1], points_3d[:, 0, 2], c='b', marker='o')
        # ax.scatter3D(camera_location[0], camera_location[1], camera_location[2], c=camera_location[2], cmap='Blues')

        # map_initialised = True

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()


if __name__ == '__main__':
    dirs = os.listdir('..')
    images_dir = list(filter(lambda directory: directory.startswith('images'), dirs))[0]
    n_files = len(os.listdir(f'../{images_dir}'))

    # draw_kp()

    # find_matching()

    slam()
