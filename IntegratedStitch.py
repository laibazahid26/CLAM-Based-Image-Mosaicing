#!/usr/bin/env python

# Python libs
import time
import math
import sys
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
from std_msgs.msg import Int16



np.set_printoptions(threshold=sys.maxint)


images = []
TimeStamps = []

ReducedImages = [] #size of all the images is reduced and appended in this array


SLAMTimeStamps = []
ReducedSLAMTimeStamps = []

SLAMkps = []
ReducedSLAMkps = []

SLAMDescs = []
ReducedSLAMDescs = []

StepSize = 5
iteration = 0



def imageCallback(image):
    
    global images, TimeStamps
    #print ('received image of type: "%s"' % image.encoding)
    TimeStamps.append((image.header.stamp).to_sec())
    
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="mono8")

    images.append(cv_image)
    


def extractFeatures_0_Callback(message_Map):
    
    global iteration
    
    kp_array = []
    desc_array = []
    
    iteration += 1
       
    if len(message_Map.Keyframes) != 0:
       
       SLAMTimeStamps.append(message_Map.Keyframes[0].dTimestamp)
       #print "id: ", frameID
       
       kp = message_Map.Keyframes[0].mvKeysUn
       #print ("length of kp: ", len(kp))
       for i in range(0, len(kp)):
          kp_array.append(cv2.KeyPoint(x=kp[i].fPoint2f_x, y=kp[i].fPoint2f_y, 
                                       _size=kp[i].size, _angle=kp[i].angle, 
                                       _response=kp[i].response, _octave=kp[i].octave))
          #print "kp_array: ", kp_array
       SLAMkps.append(kp_array)	
       
       desc = message_Map.Keyframes[0].mDescriptors
       for i in range (0, len(desc)):
          desc_array.append(list(bytearray(desc[i].mDescriptor)))
          #print "desc_array: ", desc_array
       desc_array = np.asarray(desc_array, dtype=np.uint8)
       SLAMDescs.append(desc_array)
       #print "SLAMDescs: ", SLAMDescs
      
 
def imagePreprocessing():
    
    global ReducedImages, ReducedSLAMTimeStamps, ReducedSLAMkps, ReducedSLAMDescs
    time.sleep(115)
    print("total raw images: ", len(images))

    for i in range (0, 100, 1):
       ReducedSLAMTimeStamps.append(SLAMTimeStamps[i])
       ReducedSLAMkps.append(SLAMkps[i])
       ReducedSLAMDescs.append(SLAMDescs[i])
     
    dsize = (300, 400)
     
    for i in range (0, len(images)):
       img = cv2.resize(images[i], dsize)
       ReducedImages.append(img)

    #print "time stamps: ", TimeStamps
    #print "SLAMTimeStamps: ", SLAMTimeStamps
    print "length of total received time stamps from ROS-bag: ", len(TimeStamps)
    print "length of total chosen time stamps: ", len(SLAMTimeStamps) 
    print "length of all kps from other repo: ", len(SLAMkps)
#    print "length of total extracted images: ", len(ReducedImages)
#    print "length of total extracted time stamps: ", len(ReducedTimeStamps)
    print "how many times 'map' message came: ", iteration

        
def findGoodMatches(matches):

    good = []
    for m, n in matches:
       if m.distance < 0.5 * n.distance:
          good.append(m)

    return good


def Homography(keypoints1, keypoints2, goodMatches):

    dst_pts = np.float32([ keypoints1[m.queryIdx].pt for m in goodMatches]).reshape(-1,1,2)
    src_pts = np.float32([ keypoints2[m.trainIdx].pt for m in goodMatches]).reshape(-1,1,2)
    H, _ = cv2.findHomography(src_pts, dst_pts, cv2.RHO, 5.0)
    
    return H
    
    
def warpTwoImages(index1, index2, prev_H):
        
#    print "index1: ", index1
#    print "index2: ", index2

    warpedImage = np.zeros((3700, 3700))
    
    keypoints1 = ReducedSLAMkps[index1]
    descriptors1 = ReducedSLAMDescs[index1]

    keypoints2 = ReducedSLAMkps[index2]
    descriptors2 = ReducedSLAMDescs[index2]
    
    matches = bf.knnMatch(descriptors1, descriptors2, k=2)
    goodMatches = findGoodMatches(matches)
#    print "number of good matches: ", len(goodMatches)
    H = Homography(keypoints1, keypoints2, goodMatches)
    #print H
    prev_H = np.dot(prev_H, H)
    
    selected_ts = ReducedSLAMTimeStamps[index2]
    frameNum = TimeStamps.index(selected_ts)
#    print "frame Number: ", frameNum

    warpedImage = cv2.warpPerspective(ReducedImages[frameNum], prev_H, 
                                              (warpedImage.shape[1], warpedImage.shape[0]))

    return prev_H, warpedImage


def warpEntireArray():
    
    
    warpedImages = [None]*len(ReducedSLAMTimeStamps)

    offset = [800, 1800]
    offsetMatrix = np.array([[1, 0, offset[0]], [0, 1, offset[1]], [0, 0, 1]])

    middle_image = int(math.ceil((len(ReducedSLAMTimeStamps))/2.0))
    print ("middle_image: ", middle_image)

    prev_H = offsetMatrix.copy()
    for i in range(middle_image, 0, -1):
       prev_H, warpedImage = warpTwoImages(i, i-1, prev_H)
       warpedImages[i-1] = warpedImage

    prev_H = offsetMatrix.copy()
    prev_H, warpedImage = warpTwoImages(middle_image, middle_image, prev_H)
    warpedImages[middle_image] = warpedImage

    prev_H = offsetMatrix.copy()
    for j in range(middle_image+1, len(ReducedSLAMTimeStamps)):
       prev_H, warpedImage = warpTwoImages(j-1, j, prev_H)
       warpedImages[j] = warpedImage
       
    return warpedImages


def main():
   
    global orb, bf

    sub_camera = rospy.Subscriber("/cam0/image_raw", Image, imageCallback,  queue_size=100000)
    sub_descriptors = rospy.Subscriber("/ccmslam/MapOutClient0", Map,
    					extractFeatures_0_Callback,  queue_size=100000)
    rospy.init_node('stitch_the_images')

    orb = cv2.ORB_create(nfeatures = 1000)
    bf = cv2.BFMatcher_create(cv2.NORM_HAMMING)
    

    imagePreprocessing()
    warpedImages = warpEntireArray()      

    finalImg = warpedImages[0]
    cv2.imwrite('/root/Desktop/thesis' + 'test.png', finalImg)
    
    b = Blender() 

    for i in range(1, len(warpedImages)):
       print('blending', i)
       finalImg, mask1truth, mask2truth = b.blend(finalImg, warpedImages[i])
       cv2.imwrite('/root/Desktop/thesis' + 'final.png', finalImg)
    
    rospy.spin()


if __name__ == '__main__':
    main()

