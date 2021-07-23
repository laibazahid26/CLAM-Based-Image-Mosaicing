#!/usr/bin/env python

# Python libs
import time
import struct

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

images = []

def imageCallback(image):
    
    global images

    print ('received image of type: "%s"' % image.encoding)
    
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="mono8")

    images.append(cv_image)
    

def imagePreprocessing():
    
    time.sleep(40)
    global images
    
    scale_percent = 70
    width = int(images[0].shape[1] * scale_percent / 100)
    height = int(images[0].shape[0] * scale_percent / 100)

    dsize = (width, height)

    for i in range (0, len(images)-1):
       images[i] = cv2.resize(images[i], dsize)
    
    return images


def findMatches(img1, img2):
    
    #global keypoints1, keypoints2, descriptors1, descriptors2, matches
    global orb, bf 

    keypoints1, descriptors1 = orb.detectAndCompute(img1, None)
    keypoints2, descriptors2 = orb.detectAndCompute(img2, None)

    matches = bf.knnMatch(descriptors1, descriptors2,k=2)

    return keypoints1, keypoints2, matches

 
def findGoodMatches(matches):

    good = []
    for m, n in matches:
       if m.distance < 0.5 * n.distance:
          good.append(m)

    return good


def findHomography(img1, img2, keypoints1, keypoints2, goodMatches):

    dst_pts = np.float32([ keypoints1[m.queryIdx].pt for m in goodMatches]).reshape(-1,1,2)
    src_pts = np.float32([ keypoints2[m.trainIdx].pt for m in goodMatches]).reshape(-1,1,2)
    M, _ = cv2.findHomography(src_pts, dst_pts, cv2.RHO, 5.0)

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
  
    translation_dist = [-x_min,-y_min]
  
    H_translation = np.array([[1, 0, translation_dist[0]], [0, 1, translation_dist[1]], [0, 0, 1]])

    output_img = cv2.warpPerspective(img2, H_translation.dot(H), (x_max-x_min, y_max-y_min))
    output_img[translation_dist[1]:rows1+translation_dist[1], translation_dist[0]:cols1+translation_dist[0]] = img1

    return output_img


def main():
   
    global orb, bf

    sub_camera = rospy.Subscriber("/cam0/image_raw", Image, imageCallback,  queue_size=1)
    rospy.init_node('stitch_the_images')

    orb = cv2.ORB_create(nfeatures = 1500)
    bf = cv2.BFMatcher_create(cv2.NORM_HAMMING)

    imagePreprocessing()

    result = images[0]
    #preserve_M = np.identity(3)

    print "length of images: ", len(images)
 
    #for i in range(0, len(images)-1, 10):
    version = struct.calcsize("P")*8 
    print(version)
    for i in range(0, 100, 2):
       keypoints1, keypoints2, matches = findMatches(result, images[i+1])
       goodMatches = findGoodMatches(matches) 
       if len(goodMatches) >= 10:
          print i
          M = findHomography(result, images[i+1], keypoints1, keypoints2, goodMatches)
          #preserve_M = preserve_M.dot(M) 
          result = warpTwoImages(result, images[i+1], M)

    cv2.imshow('window', result)
    cv2.waitKey(0)
    
    rospy.spin()


if __name__ == '__main__':
    main()

