#!/usr/bin/env python

# Python libs
from blendingStitchDesktop import Blender
import math

# numpy and scipy
import numpy as np
from numpy import float32, int32

import imutils

# OpenCV
import cv2


def imagePreprocessing():
    
    images = []
    
    img1 = cv2.imread("city_01.jpg")
    img2 = cv2.imread("city_02.jpg")
    img3 = cv2.imread("city_03.jpg")
    img4 = cv2.imread("city_04.jpg")
    img5 = cv2.imread("city_05.jpg")
    img6 = cv2.imread("city_06.jpg")
    img7 = cv2.imread("city_07.jpg")
    img8 = cv2.imread("city_08.jpg")

    dsize = (400,400)

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
    
    global orb, bf 

    keypoints1, descriptors1 = orb.detectAndCompute(img1, None)
    keypoints2, descriptors2 = orb.detectAndCompute(img2, None)
    
    matches = bf.knnMatch(descriptors1, descriptors2, k=2)

    return keypoints1, keypoints2, matches

 
def findGoodMatches(matches):

    good = []
    for m, n in matches:
       if m.distance < 0.5 * n.distance:
          good.append(m)

    return good
 

def Homography(keypoints1, keypoints2, goodMatches):

    dst_pts = np.float32([keypoints1[m.queryIdx].pt for m in goodMatches]).reshape(-1,1,2)
    src_pts = np.float32([keypoints2[m.trainIdx].pt for m in goodMatches]).reshape(-1,1,2)
    H, _ = cv2.findHomography(src_pts, dst_pts, cv2.RHO, 5.0)

    return H
    

def warpTwoImages(img1, img2, prev_H):
    
    warpedImage = np.zeros((4000, 4000, 3))

    keypoints1, keypoints2, matches = findMatches(img1, img2)
    goodMatches = findGoodMatches(matches)
    H = Homography(keypoints1, keypoints2, goodMatches)
    prev_H = np.dot(prev_H, H)
    warpedImage = cv2.warpPerspective(img2, prev_H, (warpedImage.shape[1], warpedImage.shape[0]))

    return prev_H, warpedImage


def main():
   
    global orb, bf, allWarpedImages
    

    orb = cv2.ORB_create(nfeatures = 1500)
    bf = cv2.BFMatcher_create(cv2.NORM_HAMMING)
   

    images = imagePreprocessing()
    allWarpedImages = [np.zeros((4000, 4000, 3))]* len(images)


    offset = [1000, 500]
    offsetMatrix = np.array([[1, 0, offset[0]], [0, 1, offset[1]], [0, 0, 1]])

    middle_image = int(math.ceil(len(images)/2))

    prev_H = offsetMatrix.copy()
    for i in range(middle_image, 0, -1):
       prev_H, warpedImage = warpTwoImages(images[i], images[i-1], prev_H)
       allWarpedImages[i-1] = warpedImage
       cv2.imwrite('/root/Desktop/thesis/' + str(i-1) + '.png', allWarpedImages[i-1])

    prev_H = offsetMatrix.copy()
    prev_H, warpedImage = warpTwoImages(images[middle_image], images[middle_image], prev_H)
    allWarpedImages[middle_image] = warpedImage
    cv2.imwrite('/root/Desktop/thesis/' + str(middle_image) + '.png', allWarpedImages[middle_image])
    
    prev_H = offsetMatrix.copy()
    for j in range(middle_image+1, len(images)):
       prev_H, warpedImage = warpTwoImages(images[j-1], images[j], prev_H)
       allWarpedImages[j] = warpedImage
       cv2.imwrite('/root/Desktop/thesis/' + str(j) + '.png', allWarpedImages[j])


    finalImg = allWarpedImages[0]
    b = Blender() 
    
    for index in range(1, len(images)):
        print('blending', index)
        finalImg, mask1truth, mask2truth = b.blend(finalImg, allWarpedImages[index])
        mask1truth = mask1truth + mask2truth
        cv2.imwrite('/root/Desktop/thesis/FINALBLENDED.png', finalImg)
        #cv2.imshow('Final Image', finalImg)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

