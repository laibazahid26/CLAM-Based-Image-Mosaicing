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
 
 
def imageCallback(image):
    
    global images
    #print ('received image of type: "%s"' % image.encoding)
    
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="mono8")

    images.append(cv_image)
    


def chosenKFCallback (msg):

    ChosenKeyFrame.append(msg.data) 


def extractFeatures_0_Callback(message_Map):

    #time.sleep(7)
    if len(message_Map.Keyframes) != 0:
       #msgid = message_Map.Keyframes[0].mUniqueId
       msgid = message_Map.header
       kp = message_Map.Keyframes[0].mvKeysUn
       desc = message_Map.Keyframes[0].mDescriptors
    
       AllKp.append(kp)
       AllDesc.append(desc)
       AllMsgId.append(msgid)
       #print (kp)
       #print (msgid)

def imagePreprocessing():
    
    global images
    time.sleep(160)
    
    allImages = []
    print "total raw images: ", len(images)
    
    extracted = images[:]
    dsize = (250, 350)
     
    for i in range (0, len(extracted), 2):
       img = cv2.resize(extracted[i], dsize)
       allImages.append(img)
    
    print ("length of chosenkeyFrame list: ", len(ChosenKeyFrame))
    return allImages


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

    dst_pts = np.float32([ keypoints1[m.queryIdx].pt for m in goodMatches]).reshape(-1,1,2)
    src_pts = np.float32([ keypoints2[m.trainIdx].pt for m in goodMatches]).reshape(-1,1,2)
    M, _ = cv2.findHomography(src_pts, dst_pts, cv2.RHO, 5.0)
    
    return M
    
def warpTwoImages(img1, img2, prev_H):
    
    warpedImage = np.zeros((3000, 2500))

    keypoints1, keypoints2, matches = findMatches(img1, img2)
    goodMatches = findGoodMatches(matches)
    H = Homography(keypoints1, keypoints2, goodMatches)
    prev_H = np.dot(prev_H, H)
    warpedImage = cv2.warpPerspective(img2, prev_H, (warpedImage.shape[1], warpedImage.shape[0]))

    return prev_H, warpedImage

def warpEntireArray(array):


    warpedImages = [None]*len(array)

    offset = [1500, 1000]
    offsetMatrix = np.array([[1, 0, offset[0]], [0, 1, offset[1]], [0, 0, 1]])

    middle_image = int(math.ceil(len(array)/2))
    print "middle_image: ", middle_image

    prev_H = offsetMatrix.copy()
    for i in range(middle_image, 0, -1):
       prev_H, warpedImage = warpTwoImages(array[i], array[i-1], prev_H)
       warpedImages[i-1] = warpedImage
       #cv2.imwrite('/root/Desktop/thesis/' + str(i-1) + '.png', warpedImages[i-1])

    prev_H = offsetMatrix.copy()
    prev_H, warpedImage = warpTwoImages(array[middle_image], array[middle_image], prev_H)
    warpedImages[middle_image] = warpedImage
    #cv2.imwrite('/root/Desktop/thesis/' + str(middle_image) + '.png', warpedImages[middle_image])
    
    prev_H = offsetMatrix.copy()
    for j in range(middle_image+1, len(array)):
       prev_H, warpedImage = warpTwoImages(array[j-1], array[j], prev_H)
       warpedImages[j] = warpedImage
       #cv2.imwrite('/root/Desktop/thesis/' + str(j) + '.png', warpedImages[j])

    
#    iterator = 0
#    for i in range(0, len(allWarpedImages)):
#       for j in range(0, len(allWarpedImages[i])):
#          cv2.imwrite('/root/Desktop/thesis/' + str(iterator) + '.png', allWarpedImages[i][j])
#          iterator +=1

    return warpedImages

def main():
   
    global orb, bf

    sub_camera = rospy.Subscriber("/cam0/image_raw", Image, imageCallback,  queue_size=1000)
    sub_chosenKF = rospy.Subscriber("/ChosenKeyFrame", Int8, chosenKFCallback,  queue_size=1000)
    sub_descriptors = rospy.Subscriber("/ccmslam/MapOutClient0", Map,
    					extractFeatures_0_Callback,  queue_size=1000)
    rospy.init_node('stitch_the_images')

    orb = cv2.ORB_create(nfeatures = 1500)
    bf = cv2.BFMatcher_create(cv2.NORM_HAMMING)

    images = imagePreprocessing()
#    warpedImages = warpEntireArray(images)
          

#    finalImg = warpedImages[0]
#    b = Blender() 

#    for i in range(1, len(warpedImages)):
#       print('blending', i)
#       finalImg, mask1truth, mask2truth = b.blend(finalImg, warpedImages[i])
#       cv2.imwrite('/root/Desktop/thesis' + 'final.png', finalImg)
    
    rospy.spin()


if __name__ == '__main__':
    main()

