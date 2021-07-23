# VisualColaborativeMapping

I have uploaded two py files. One is called StitchDesktop.py and the other is called Stitch.py. The former one is for the testing purposes. I have used a set of 8 different pictures of the city for testing my algorithm and checking the result of algorithm on this set of pictures.
<br />
<br />
Whereas, the second py file, 'Stitch.py' uses the rosbag data to build a panorama. 

###  Running Instructions
It is suggested to run first StitchDesktop.py file for having a look at the code and analyzing the problems in the resultant panaorama. This file can simply be run by typing: <br />
`./StitchDesktop.py`

For running the second py file below are the steps: <br />
First download the rosbag named 'Machine Hall 01' from the link below: <br />
https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

In one terminal type: <br />
`./Stitch.py`

Open second terminal and type: <br />
`rosbag play MH_01_easy.bag --start 45`

Stop the rosbag by pressing ctrl-C after 20 to 25 seconds (because we are testing only). <br />
Also, one important thing is that I am having memory issues in my code (Stitch.py), if I run it for more than 50 images. 
