# Self Driving Car Nanodegree - Capstone Project: System Integration by Team Sakura
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

![carla](./imgs/udacity-self-driving-car.jpeg)

## Results
### 3rd submission (Tag 1.2.0)
Carla was creeping, not full stop, when the traffic light was **red**.  
When the traffic light turned **green**, Carla had a smooth acceleration.  

[![Real Sefl-Driving Car](http://img.youtube.com/vi/mmI-CeORwAM/0.jpg)](https://www.youtube.com/watch?v=mmI-CeORwAM)

The DBW node outputs `/vehicle/brake_cmd`, but Carla does not stop at the stop position.  
Maybe the brake torque is not enough.

|`current_velocity`, `throttle_cmd` and `brake_cmd`|`/base_waypoints` and `/current_pose`|
|:--:|:--:|
|![](./imgs/vel_throttle_brake_new.png)|![](./imgs/wp_and_pose_zoom_new.png)|

### 2nd submission (Tag 1.1.0)
The car did not stop even though the traffic light was **red**.  
The DBW node outputs `/vehicle/brake_cmd`, but Carla does not stop at the stop position.  
The cause is that it accelerates too much at first, and the transition to deceleration is slow.

|`current_velocity`, `throttle_cmd` and `brake_cmd`|`/base_waypoints` and `/current_pose`|
|:--:|:--:|
|![](./imgs/vel_throttle_brake.png)|![](./imgs/wp_and_pose_zoom.png)|

### For 2nd submission
We limit the velocity toward the stop line to approximately 10 km/h and improve the detection performance for blurred images.
- Change `MAX_DECEL` from 5.0 to 0.5 in `waypoint_updater.py`.
- Change `real_model.h5` used by `tl_detector.py` (but not included in repository).

|`MAX_DECEL` and decelerate_vel|`real_model.h5` (1.1.0)|
|:--:|:--:|
|![](./imgs/decel_vel.png)|![](./imgs/tl_site_new.jpg)|

### 1st submission (Tag 1.0.0)
The car did not stop even though the traffic light was **red**.  
The cause is that the blurred traffic lights are can not be detected when the velocity exceeds about 10 km/h on the way to `/traffic_waypoint`.

|`/current_velocity` and `/traffic_waypoint`|`real_model.h5` (1.0.0)|
|:--:|:--:|
|![](./imgs/cv_and_tw_zoom.png)|![](./imgs/tl_site_old.jpg)|

## Team Sakura

|Night|Day|
|:--:|:--:|
|![](./imgs/night.png)|![](./imgs/day.png)|

### Team Members
- Hidetoshi Furukawa: hidetoshi.furukawa (at) ai4sig.com **team lead**
- João Gonçalves: miguel.joao.goncalves (at) gmail.com
- Liangli Fei: fei.liangli.info (at) gmail.com
- Yuji Kawamura: yujika2019 (at) gmail.com
- Junxun Luo: luojunxun (at) gmail.com

## Submission checklist and requirements
- Launch correctly using the launch files provided in the capstone repo.
- Smoothly follow waypoints in the simulator.
- Respect the target top speed set for the waypoints' `twist.twist.linear.x` in `waypoint_loader.py`.
- Stop at traffic lights when needed.
- Stop and restart PID controllers depending on the state of `/vehicle/dbw_enabled`.
- Publish throttle, steering, and brake commands at 50Hz.

## System Architecture
The following is a system architecture diagram showing the ROS nodes and topics used in the project.
The ROS nodes and topics shown in the diagram are described briefly in the **Code Structure** section below.

![System Architecture](imgs/final-project-ros-graph-v2.png)

## Code Structure
Below is a brief overview of the repo structure, along with descriptions of the ROS nodes.

### (path_to_project_repo)/ros/src/tl_detector/
This package contains the traffic light detection node: `tl_detector.py`.
This node takes in data from the `/image_color`, `/current_pose`, and `/base_waypoints` topics and publishes the locations to stop for red traffic lights to the `/traffic_waypoint` topic.

The `/current_pose` topic provides the vehicle's current position, and `/base_waypoints` provides a complete list of waypoints the car will be following.

We build both a traffic light detection node and a traffic light classification node. Traffic light detection should take place within `tl_detector.py`, whereas traffic light classification should take place within `../tl_detector/light_classification_model/tl_classfier.py`.

![Traffic Light Detection Node](imgs/tl-detector-ros-graph.png)

See code in
[/ros/src/tl_detector/](https://github.com/hidetoshi-furukawa/CarND-Capstone/tree/master/ros/src/tl_detector/).

### (path_to_project_repo)/ros/src/waypoint_updater/
This package contains the waypoint updater node: `waypoint_updater.py`.

![Waypoint Updater Node](imgs/waypoint-updater-ros-graph.png)

See code in [/ros/src/waypoint_updater/](https://github.com/hidetoshi-furukawa/CarND-Capstone/tree/master/ros/src/waypoint_updater/).

### (path_to_project_repo)/ros/src/twist_controller/
Carla is equipped with a drive-by-wire (DBW) system, meaning the throttle, brake, and steering have electronic control.
This package contains the files that are responsible for control of the vehicle: the node `dbw_node.py` and the file `twist_controller.py`.

![DBW Node](imgs/dbw-node-ros-graph.png)

See code in [/ros/src/twist_controller/](https://github.com/hidetoshi-furukawa/CarND-Capstone/tree/master/ros/src/twist_controller/).

#### (path_to_project_repo)/ros/src/styx/
A package that contains a server for communicating with the simulator, and a bridge to translate and publish simulator messages to ROS topics.
#### (path_to_project_repo)/ros/src/styx_msgs/
A package which includes definitions of the custom ROS message types used in the project.
#### (path_to_project_repo)/ros/src/waypoint_loader/
A package which loads the static waypoint data and publishes to `/base_waypoints`.
#### (path_to_project_repo)/ros/src/waypoint_follower/
A package containing code from Autoware which subscribes to `/final_waypoints` and publishes target vehicle linear and angular velocities in the form of twist commands to the `/twist_cmd` topic.

## Traffic Light Detection
### Model for Simulator
We use Tiny YOLOv3 for traffic light detection and classification.

1. Download Tiny YOLOv3 (also called YOLOv3-tiny) weight from [YOLO website](http://pjreddie.com/darknet/yolo/)
1. Convert the Darknet YOLO model to a Keras model using [keras-yolo3](https://github.com/hidetoshi-furukawa/keras-yolo3).
1. Transfer learning for traffic light detection.

|Loss|Val Loss|
|:---|:---|
|![](imgs/sim_loss.png)|![](imgs/sim_val_loss.png)|

Graph of the Tiny YOLOv3 model:
<div class="test">
<img src="imgs/yolo_tiny_graph.png" width="600">
</div>

### Model for Site
We use YOLOv3 for traffic light detection and classification.

YOLOv3 (open youtube video on click):

[![YOLOv3 for site](imgs/tl_for_site.jpg)](https://www.youtube.com/watch?v=rLzaqbla7yA)

Tiny YOLOv3 (open youtube video on click):

[![Tiny YOLOv3 for site](imgs/tl_tiny_for_site.jpg)](https://www.youtube.com/watch?v=sHTn8sFlQEw)

## Original Instructions
This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

- Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
- If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  - 2 CPU
  - 2 GB system memory
  - 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

- Follow these instructions to install ROS
  - [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  - [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
- [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  - Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
- Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
