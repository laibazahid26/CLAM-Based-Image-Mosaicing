# CLAM Based Image Mosaicing

### The aim 
The aim of this work is to build a 2D map of the site or a mosaic of an unknown environment. This work is targeted at a search and rescue mission. This aim is accomplished by integrating this work with the CCM-SLAM system[1]. Even though the 3D map produced by CCM-SLAM is useful for the robot to perform autonomous navigation, but humans might face difficulties to understand this 3D map and it does not provide a lot of information to the human eye, even if the map is dense or semi-dense. 
<br />
<br />
Human responders, at an earthquake site, search for human lives randomly, and it is possible that some life exists in a nearby area where human responders are currently searching and there are chances that that life will be lost due to delay in rescue.
<br />
<br />
Since time is of crucial importance in any search and rescue mission, a more accustomed view of the human eyes will definitely help make important decisions in less time. 
<br />
<br />
The aim of this work is to use features, extracted by the CCM-SLAM system using the sensors of various UAVs involved in the mission, and reconstruct a clear mosaic of the large environment with respect to the drone positions.
<br />
I have uploaded two py files. One is called StitchDesktop.py and the other is called Stitch.py. The former one is for the testing purposes. I have used a set of 8 different pictures of the city for testing my algorithm and checking the result of algorithm on this set of pictures.
<br />
<br />
Whereas, the second py file, 'Stitch.py' uses the rosbag data to build a panorama. This file extracts the features on its own instead of taking it from CCM-SLAM system.
<br />
<br />
Finally, the third py file, 'IntegratedStitch.py' can be run to see the working of the whole system. The steps for ruuning this file are given below. 
<br />
<br />
The image blending module has been taken from the reference[2] provided below. For visualizing the bleneded image in rviz please refer to work done in refernce[3]. This
package lets me view the mosaic by building a customised plugin to be used in rviz. This customised plugin converts the images into meshes so that they could be visualized on the rviz grid as a marker. I exported this plugin into the rviz of CCM-SLAM. The results could be seen below:  

![alt text](https://github.com/laibazahid26/CLAM-Based-Image-Mosaicing/blob/master/IntegratedResults2.JPG?raw=true)

###  Running Instructions
1. Download and Install Ubuntu or Docker. 
2. Install and build the CCM-SLAM system by carefully following the instructions given
on the readme file of the following github repository.
https://github.com/VIS4ROB-lab/ccm_slam
3. Download the file name IntegratedStitch.py from the following repository.
https://github.com/laibazahid26/CLAM-Based-Image-Mosaicing/tree/master
4. Make a folder named scripts inside the src folder of ccm-slam package and paste the
IntegratedStitch.py file in that folder.
5. Launch the rosbag, servers and the clients as described in the readme file of step 2.
6. launch the IntegratedStitch.py file by going inside the directory where this file is placed
from terminal and typing:
python2 IntegratedStitch.py

### References
[1]. https://github.com/VIS4ROB-lab/ccm_slam<br />
[2]. https://github.com/praveenVnktsh/Panorama-Stitching<br />
[3]. https://github.com/lucasw/rviz_textured_quads<br />
