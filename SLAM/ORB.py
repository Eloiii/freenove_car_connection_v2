import numpy as np
import cv2 as cv
import os


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
        print(f'images {k}.jpg and {k+1}.jpg')
        img1 = cv.imread(f'../{images_dir}/{k}.jpg', cv.IMREAD_GRAYSCALE)
        img2 = cv.imread(f'../{images_dir}/{k+1}.jpg', cv.IMREAD_GRAYSCALE)

        kp1, des1 = orb.detectAndCompute(img1, None)
        kp2, des2 = orb.detectAndCompute(img2, None)

        bf = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)
        matches = bf.match(des1, des2)
        matches = sorted(matches, key=lambda x: x.distance)

        img3 = cv.drawMatches(img1, kp1, img2, kp2, matches[:10], None,
                              flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        cv.imwrite(f'./{orb_dir}/{k}-{k+1}.jpg', img3)


if __name__ == '__main__':
    dirs = os.listdir('..')
    images_dir = list(filter(lambda directory: directory.startswith('images'), dirs))[0]
    n_files = len(os.listdir(f'../{images_dir}'))

    # draw_kp()

    find_matching()


