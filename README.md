# VisualColaborativeMapping

First download the rosbag named 'Machine Hall 01' from the link below: 
https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

In one terminal type: <br />
./Stitch.py

Open second terminal and type: <br />
rosbag play MH_01_easy.bag --start 45

Stop the rosbag by pressing ctrl-C after 20 to 25 seconds (because we are testing only). 
