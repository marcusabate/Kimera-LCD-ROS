# NOTE: Deprecated

This package does not work with the most up-to-date Kimera and at this point is only intended as an example of how to implement a ROS wrapper for the LCD module of Kimera.

# Kimera-LCD-ROS

This is a ROS wrapper for testing the `LoopClosureDetector` tools for Kimera-VIO.
It allows testing the `LoopClosureDetector` without running the entire VIO
pipeline.

While not recommended, it is possible to run this node in parallel with other VIO ROS modules as a standalone loop-closure-detector module. This can be done by adding to other VIO libraries the ability to publish `LcdInputPayload` structures to the input topic of this node. This is not recommended because it will not be as fast as adding the `LoopClosureDetector` directly in VIO pipelines (as [`Kimera-VIO`](https://github.com/MIT-SPARK/Kimera-VIO) does) owing to the networking overhead of ROS.

The package requires `Kimera-VIO`, which in turn has several other dependencies
including gtsam, OpenCV, and OpenGV.

## Dependencies

Install [Kimera-VIO](https://github.com/MIT-SPARK/Kimera-VIO) and all its dependencies
by following the install instructions found in that repo.

## Installation

Clone this repo into a catkin ws:

```bash
cd ~/catkin_ws/src
git clone https://github.com/marcusabate/Kimera-LCD-ROS.git
```

If you have `Kimera-VIO` and all its dependencies, you can build the workspace now. Otherwise, use the provided rosinstall files to automatically download them:

```bash
wstool init
wstool merge Kimera-LCD-ROS/install/kimera_lcd_ros_http.rosinstall
# wstool merge Kimera-LCD-ROS/install/kimera_lcd_ros_ssh.rosinstall  # Use this if you don't have ssh keys.
wstool update

catkin build kimera_lcd_ros
```

## Features

* The [`kimera_lcd_ros_node`](/src/kimera-lcd-ros-node.cpp) node is the main node that spins the
LoopClosureDetector pipeline. It expects `LcdInputPayload`s published to an input topic, which is handled externally. It will store these images in a local database and detect loop closures between similar images.

* The node outputs an image including both frames involved in the loop closure
as well as the feature correspondences drawn between the frames. All features
are visualized as blue circles, while the correspondences selected to perform
pose recovery have lines drawn between them.

* The node also outputs a [`Closure`](/msg/Closure.msg) message containing
information regarding the loop closure detection, including the calculated
relative pose.

* The [`closure_visualizer_node`](/scripts/closure_visualizer_node.py) node is
used to visualize the loop closure results in RVIZ. The node outputs the error
in the calculated relative pose as compared to ground truth, which is published
to the tf tree by the [`transform_converter_node`](/scripts/transform_converter_node.py).

* The node also outputs a `geometry_msgs::PoseArray` and
`visualization_msgs::MarkerArray` type message that can be visualized in RVIZ
if ground truth pose information is available. The ground-truth reference frame
pose for each loop closure is sent to RVIZ, along with the relative pose of
the current frame for each loop closure and the ground-truth pose of that
current frame. These pose are all visualized as yellow arrows.

* The `MarkerArray` includes green arrows showing the connection between the
reference frame and the computed current frame pose. The red arrows connect
that computed current frame pose with the ground-truth current frame, as a
quick means to visualize error. In general, large red arrows means high error in the pose recovery.

`LoopClosureDetector` requires a custom input payload on an input rostopic of the form [`LcdInputPayload`](/msg/LcdInputPayload.msg). This includes a `StereoFrame` (which has two images and other parameters) as well as a VIO guess on the pose of that frame in the world reference frame. In `Kimera`, this is provided via the VIO module. Because this repo is VIO-agnostic, you need something to provide these VIO poses to the pose graph.

This can be done internally with the [`ros_lcd_data_provider`](/scripts/ros_lcd_data_provider.py) node, which parses rosbag input from arbitrary stereo datasets and creates the `LcdInputPayload` structure. It provides an empty pose-estimate, but this can easily be modified to provide any kind of guess.

## Usage

To run with one of the EuRoC datasets, simply run
```bash
roslaunch kimera_lcd_ros lcd_ros_euroc.launch
```
and then run the rosbag **once the vocabulary completely loads.** (A message
will be printed to console when this is finished; it will take around 10
seconds).

Note that this quick setup works out-of-the-box with EuRoC rosbags recorded
from their Vicon MoCap room **only**.

For other datasets some additional setup is required. Start by making a launch
file suitable with all of the required topic remapping. In particular, make
sure that the topics for the camera images are remapped properly.

In order to visualize and calculate error properly, ground truth transforms
are required. Many datasets will send this straight to the `/tf` tree.
Change the `body_frame_id` argument accordingly. However, for datasets like
EuRoC which send the transform to a normal rostopic instead, change the
`gnd_truth_topic` and include the
[`transform_converter_node`](/scripts/transform_converter_node.py) as in the
example launch file. This node simply publishes the transforms from the topic
to the `/tf` tree.

In addition, parameters for the camera must be defined. Make a new folder
in [`/param`](/param) entitled with the name of your dataset and include
a [`calibration.yaml`](/param/EUROC/calibration.yaml) file detailing the
camera parameters.

You can then change the dataset name in your launch file and the redirection
will be done automatically. In this parameter folder, you can also define
custom parameters for the LoopClosureDetector module.

## ROS Launch Arguments

* `bool log_output` is sent to LoopClosureDetector for sending output to a log file
* `float cache_time` is sent to the closure visualizer node. It represents the length of the cache for the transform listener buffer. This should be set to the length of the rosbag dataset
* `bool visualize` is for starting and publishing to RVIZ

## ToDo

- [ ] Support for offline rosbag parsing
- [ ] Speed up image transport for `LcdInputPayload`s
- [ ] Streamline visualization features
