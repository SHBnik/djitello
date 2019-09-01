# djitello
this is ROS driver for DJI tello based on DJI python SDK

## Installing
before everything run this two lines to dont get fucking "core dump" error
```
sudo apt-get install libtcmalloc-minimal4
export LD_PRELOAD="/usr/lib/libtcmalloc_minimal.so.4"
```
install FFmpeg
```
sudo add-apt-repository ppa:jonathonf/ffmpeg-4
sudo apt-get update
sudo apt-get install ffmpeg
```
go to catkin_ws/src directory
```
cd catkin_ws/src
```
then clone the djitello ROS package
```
git clone https://github.com/SHBnik/djitello.git
```
go back to the catkin_ws directory
```
cd ..
```
then make catkin
```
catkin_make
```

## Run the node
after that connect to your tello wifi and run the code
```
rosrun djitello tello_node.py
```

## Test's 

### Camera Test
for view the tello's camera after running the tello_node in a new terminal run this 
```
rosrun image_view image_view image:=tello_node/image_raw
```

### Not complete
