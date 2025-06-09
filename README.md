# Mapping-with-T265-and-2dLidar-ROS2


first of all we need to build our docker(this is just for first time):

```
-- docker build -f ros2_slam.dockerfile -t ros2_slam .
```


then: 

```
-- xhost +local:root
```

and then we should run our builded docker: 


```
docker run --rm -it \
  --name ros2_slam_gui \
  --privileged \
  --net host \
  -e ROS_DOMAIN_ID=17 \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/ros2_ws/config:/ros2_ws/config:ro \
  -v ~/ros2_ws/mapping_docker:/ros2_ws/src/mapping_docker \
  --device=/dev/bus/usb/004/006 \
  ros2_slam
```




and then for launch the mapping:

```
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
```
```
cd /ros2_ws
colcon build --packages-select mapping_docker
source install/setup.bash
```
and for final step we need to run our launch file: 

```
ros2 launch mapping_docker slam.launch.py
```
------------------------------------------------------------------------------------

if you get this following error you have to launch the node again

*** Message Filter dropping message: frame 'odom' at time  for reason 'discarding message because the queue is full' ***

------------------------------------------------------------------------------------




or we should have extra 6 terminal and:


-- docker exec -it ros2_slam_gui bash


-- source /opt/ros/humble/setup.bash
 source /ros2_ws/install/setup.bash
 
 
 and then run this commands properly in each window: 


-- ros2 run realsense2_camera realsense2_camera_node    --ros-args      -p enable_pose:=true      -p publish_odom_tf:=true      -p base_frame_id:=base_link      -p odom_frame_id:=odom



-- ros2 launch urg_node2 urg_node2.launch.py   serial_port:=/dev/ttyACM0 


-- ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link


-- ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 base_link laser



-- ros2 launch robot_localization ekf.launch.py   params_file:=/ros2_ws/config/ekf.yaml


-- ros2 launch slam_toolbox online_async_launch.py   slam_params_file:=/ros2_ws/config/slam_toolbox_params.yaml



