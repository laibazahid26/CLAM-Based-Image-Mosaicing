#!/usr/bin/env python

# Python libs
import time
import math
from blendingStitch import Blender


# numpy and scipy
import numpy as np
from numpy import float32, int32

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
    
    global images
    time.sleep(30)
    
    allImages = []
    print "total raw images: ", len(images)
    firstTen = images[0:60]

    print len(firstTen)
    dsize = (300, 300)
     
    for i in range (0, len(firstTen), 2):
       img = cv2.resize(firstTen[i], dsize)
       allImages.append(img)
    
    return allImages


def findMatches(img1, img2):
    
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


def Homography(keypoints1, keypoints2, goodMatches):

    dst_pts = np.float32([ keypoints1[m.queryIdx].pt for m in goodMatches]).reshape(-1,1,2)
    src_pts = np.float32([ keypoints2[m.trainIdx].pt for m in goodMatches]).reshape(-1,1,2)
    M, _ = cv2.findHomography(src_pts, dst_pts, cv2.RHO, 5.0)
    
    #print M 
    return M
    
def warpTwoImages(img1, img2, prev_H):
    
    warpedImage = np.zeros((2000, 2000))

    keypoints1, keypoints2, matches = findMatches(img1, img2)
    goodMatches = findGoodMatches(matches)
    H = Homography(keypoints1, keypoints2, goodMatches)
    prev_H = np.dot(prev_H, H)
    warpedImage = cv2.warpPerspective(img2, prev_H, (warpedImage.shape[1], warpedImage.shape[0]))

    return prev_H, warpedImage


def main():
   
    global orb, bf

    sub_camera = rospy.Subscriber("/cam0/image_raw", Image, imageCallback,  queue_size=1)
    rospy.init_node('stitch_the_images')

    orb = cv2.ORB_create(nfeatures = 1500)
    bf = cv2.BFMatcher_create(cv2.NORM_HAMMING)

    allImages = imagePreprocessing()

    allWarpedImages = [np.zeros((2000, 2000))]* len(allImages)


    offset = [1000, 500]
    offsetMatrix = np.array([[1, 0, offset[0]], [0, 1, offset[1]], [0, 0, 1]])

    middle_image = int(math.ceil(len(allImages)/2))
    print "middle_image: ", middle_image

    prev_H = offsetMatrix.copy()
    for i in range(middle_image, 0, -1):
       prev_H, warpedImage = warpTwoImages(allImages[i], allImages[i-1], prev_H)
       allWarpedImages[i-1] = warpedImage
       #cv2.imwrite('/root/Desktop/thesis/' + str(i-1) + '.png', allWarpedImages[i-1])

    prev_H = offsetMatrix.copy()
    prev_H, warpedImage = warpTwoImages(allImages[middle_image], allImages[middle_image], prev_H)
    allWarpedImages[middle_image] = warpedImage
    #cv2.imwrite('/root/Desktop/thesis/' + str(middle_image) + '.png', allWarpedImages[middle_image])
    
    prev_H = offsetMatrix.copy()
    for j in range(middle_image+1, len(allImages)):
       prev_H, warpedImage = warpTwoImages(allImages[j-1], allImages[j], prev_H)
       allWarpedImages[j] = warpedImage
       #cv2.imwrite('/root/Desktop/thesis/' + str(j) + '.png', allWarpedImages[j])


    finalImg = allWarpedImages[0]
    b = Blender() 
    
    for index in range(1, len(allWarpedImages)):
        print('blending', index)
        finalImg, mask1truth, mask2truth = b.blend(finalImg, allWarpedImages[index])
        mask1truth = mask1truth + mask2truth
        cv2.imwrite('/root/Desktop/thesis/FINALBLENDED.png', finalImg)
    
    rospy.spin()


if __name__ == '__main__':
    main()

