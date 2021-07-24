#!/usr/bin/env python

# Python libs
import time

# numpy and scipy
import numpy as np
from numpy import float32, int32
from scipy.ndimage import filters

import imutils

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def imagePreprocessing():
    
    global img1, img2, img3, images
    images = []
    
    img1 = cv2.imread("city_01.jpg")
    img2 = cv2.imread("city_02.jpg")
    img3 = cv2.imread("city_03.jpg")
    img4 = cv2.imread("city_04.jpg")
    img5 = cv2.imread("city_05.jpg")
    img6 = cv2.imread("city_06.jpg")
    img7 = cv2.imread("city_07.jpg")
    img8 = cv2.imread("city_08.jpg")

    scale_percent = 40

    width = int(img1.shape[1] * scale_percent / 100)
    height = int(img1.shape[0] * scale_percent / 100)

    dsize = (width, height)

    img1 = cv2.resize(img1, dsize)
    img2 = cv2.resize(img2, dsize)
    img3 = cv2.resize(img3, dsize)
    img4 = cv2.resize(img4, dsize)
    img5 = cv2.resize(img5, dsize)
    img6 = cv2.resize(img6, dsize)
    img7 = cv2.resize(img7, dsize)
    img8 = cv2.resize(img8, dsize)
   
    images.append(img1)
    images.append(img2)
    images.append(img3)
    images.append(img4)
    images.append(img5)
    images.append(img6)
    images.append(img7)
    images.append(img8)

    return images


def findMatches(img1, img2):
    
    global keypoints1, keypoints2, descriptors1, descriptors2, matches
    global orb, bf 

    orb = cv2.ORB_create(nfeatures = 1500)
    keypoints1, descriptors1 = orb.detectAndCompute(img1, None)
    keypoints2, descriptors2 = orb.detectAndCompute(img2, None)
    
    bf = cv2.BFMatcher_create(cv2.NORM_HAMMING)
    matches = bf.knnMatch(descriptors1, descriptors2,k=2)

    return keypoints1, keypoints2, matches

 
def findGoodMatches(matches):

    good = []
    for m, n in matches:
       if m.distance < 0.6 * n.distance:
          good.append(m)

    return good


def Homography(img1, img2, keypoints1, keypoints2, goodMatches):

    dst_pts = np.float32([ keypoints1[m.queryIdx].pt for m in goodMatches]).reshape(-1,1,2)
    src_pts = np.float32([ keypoints2[m.trainIdx].pt for m in goodMatches]).reshape(-1,1,2)
    M, _ = cv2.findHomography(src_pts, dst_pts, cv2.RHO, 5.0)

    print "M: ", M 
    return M
    

def warpTwoImages(img1, img2, H):
  
    rows1, cols1 = img1.shape[:2]
    rows2, cols2 = img2.shape[:2]

    pts1 = np.float32([[0,0], [0, rows1],[cols1, rows1], [cols1, 0]]).reshape(-1, 1, 2)
    pts2_temp = np.float32([[0,0], [0,rows2], [cols2,rows2], [cols2,0]]).reshape(-1,1,2)

    pts2 = cv2.perspectiveTransform(pts2_temp, H)

    pts = np.concatenate((pts1,pts2), axis=0)

    [x_min, y_min] = np.int32(pts.min(axis=0).ravel() - 0.5)
    [x_max, y_max] = np.int32(pts.max(axis=0).ravel() + 0.5)
    
    output_img = cv2.warpPerspective(img2, H, (x_max-x_min, y_max-y_min))
    output_img[0:rows1, 0:cols1] = img1

    return output_img


def main():
   
    images = imagePreprocessing()

    result = images[0]
    #preserve_M = np.identity(3)

    for i in range(0, len(images)-1):
       keypoints1, keypoints2, matches = findMatches(result, images[i+1])
       goodMatches = findGoodMatches(matches)
       if len(goodMatches) >= 10:
          M = Homography(result, images[i+1], keypoints1, keypoints2, goodMatches)
          #preserve_M = preserve_M.dot(M) 
          result = warpTwoImages(result, images[i+1], M)

    cv2.imshow('window', result)
    cv2.waitKey(0)

if __name__ == '__main__':
    main()

