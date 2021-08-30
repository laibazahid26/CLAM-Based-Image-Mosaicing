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
from ccmslam_msgs.msg import Map, MP, Descriptor
from visualization_msgs.msg import Marker
from std_msgs.msg import Int8


images = []
AllKp = []
AllDesc = []
AllMsgId = []
ChosenKeyFrame = []
StepSize = 2 
 
def imageCallback(image):
    
    global images
    #print ('received image of type: "%s"' % image.encoding)
    
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="mono8")

    images.append(cv_image)
    

def chosenKFCallback (msg):

    ChosenKeyFrame.append(msg.data) 


def extractFeatures_0_Callback(message_Map):
    
    kp_array = []
    desc_array = []
    
    if len(message_Map.Keyframes) != 0:

       kp = message_Map.Keyframes[0].mvKeysUn
       print ("length of kp: ", len(kp))
       #time.sleep(3)
       for i in range(0, len(kp)):
          kp_array.append(cv2.KeyPoint(x=kp[i].fPoint2f_x, y=kp[i].fPoint2f_y, _size=kp[i].size))
          #print "kp_array: ", kp_array
       AllKp.append(kp_array)	
       
       desc = message_Map.Keyframes[0].mDescriptors
       for i in range (0, len(desc)):
          desc_array.append(list(bytearray(desc[i].mDescriptor)))
          #print "desc_array: ", desc_array
       desc_array = np.asarray(desc_array, dtype=np.uint8)
       AllDesc.append(desc_array)
       #print "AllDesc: ", AllDesc
      
       
def imagePreprocessing():
    
    global images, StepSize
    time.sleep(3600)
    
    allImages = []
    print ("total raw images: ", len(images))
    
    extracted = images[:]
    dsize = (250, 350)
     
    for i in range (0, len(extracted), StepSize):
       img = cv2.resize(extracted[i], dsize)
       allImages.append(img)
    
    print "ChosenKeyFrame: ", ChosenKeyFrame
    print "Type of ChosenKeyFrame: ", type(ChosenKeyFrame)
    print "Type of ChosenKeyFrame[0]: ", type(ChosenKeyFrame[0])
    print "length of chosenkeyFrame list: ", len(ChosenKeyFrame)
    print "number of ones in ChosenKeyFrame: ", ChosenKeyFrame.count(1)
    print "length of all kps from other repo: ", len(AllKp)
    print "length of all descs from other repo: ", len(AllDesc)
    return allImages


def findKPandDesc(img):
    
    global orb, bf 

    keypoints, descriptors = orb.detectAndCompute(img, None)

    return keypoints, descriptors

 
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
    
    return M
    
    
def warpTwoImages(img1, img2, prev_H, index1, index2):
    
    warpedImage = np.zeros((3000, 2500))
    
    index1 = index1 * StepSize
    index2 = index2 * StepSize
    
    print "index1: ", index1
    print "index2: ", index2
    
    if index1 < len(ChosenKeyFrame) and ChosenKeyFrame[index1] != 0:
       corress_index = ChosenKeyFrame[:index1+1].count(1)
       print "corress_index1 :", corress_index-1 
       keypoints1 = AllKp[corress_index-1]
       descriptors1 = AllDesc[corress_index-1]
       print "a"
    else: 
       keypoints1, descriptors1 = findKPandDesc(img1)
       print "b"
    
    if index2 < len(ChosenKeyFrame) and ChosenKeyFrame[index2] != 0: 
       corress_index = ChosenKeyFrame[:index2+1].count(1)
       print "corress_index2 :", corress_index-1
       keypoints2 = AllKp[corress_index-1]
       descriptors2 = AllDesc[corress_index-1]
       print "c"
    else: 
       keypoints2, descriptors2 = findKPandDesc(img2)
       print "d"
    
    matches = bf.knnMatch(descriptors1, descriptors2, k=2)
    print "len matches: ", len(matches)
    goodMatches = findGoodMatches(matches)
    print "len goodMatches", len(goodMatches)
    print "len keypoints1", len(keypoints1)
    print "len keypoints2", len(keypoints2)
    H = Homography(keypoints1, keypoints2, goodMatches)
    print "H: ", H
    prev_H = np.dot(prev_H, H)
    print "prev_H: ", prev_H
    warpedImage = cv2.warpPerspective(img2, prev_H, (warpedImage.shape[1], warpedImage.shape[0]))

    return prev_H, warpedImage


def warpEntireArray(array):

    warpedImages = [None]*len(array)

    offset = [1500, 1000]
    offsetMatrix = np.array([[1, 0, offset[0]], [0, 1, offset[1]], [0, 0, 1]])

    middle_image = int(math.ceil(len(array)/2))
    print ("middle_image: ", middle_image)

    prev_H = offsetMatrix.copy()
    for i in range(middle_image, 0, -1):
       prev_H, warpedImage = warpTwoImages(array[i], array[i-1], prev_H, i, i-1)
       warpedImages[i-1] = warpedImage

    prev_H = offsetMatrix.copy()
    prev_H, warpedImage = warpTwoImages(array[middle_image], array[middle_image], 
                                        prev_H, middle_image, middle_image)
    warpedImages[middle_image] = warpedImage

    prev_H = offsetMatrix.copy()
    for j in range(middle_image+1, len(array)):
       prev_H, warpedImage = warpTwoImages(array[j-1], array[j], prev_H, j-1, j)
       warpedImages[j] = warpedImage
    
    return warpedImages


def main():
   
    global orb, bf

    sub_camera = rospy.Subscriber("/cam0/image_raw", Image, imageCallback,  queue_size=100000)
    sub_chosenKF = rospy.Subscriber("/ChosenKeyFrame", Int8, chosenKFCallback,  queue_size=100000)
    sub_descriptors = rospy.Subscriber("/ccmslam/MapOutClient0", Map,
    					extractFeatures_0_Callback,  queue_size=100000)
    rospy.init_node('stitch_the_images')

    orb = cv2.ORB_create(nfeatures = 1000)
    bf = cv2.BFMatcher_create(cv2.NORM_HAMMING)

    images = imagePreprocessing()
    warpedImages = warpEntireArray(images)
          

#    finalImg = warpedImages[0]
#    b = Blender() 

#    for i in range(1, len(warpedImages)):
#       print('blending', i)
#       finalImg, mask1truth, mask2truth = b.blend(finalImg, warpedImages[i])
#       cv2.imwrite('/root/Desktop/thesis' + 'final.png', finalImg)
    
    rospy.spin()


if __name__ == '__main__':
    main()

