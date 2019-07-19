# LCD_ros

This is a ROS wrapper for testing the LoopClosureDetector tools for Spark VIO.
It allows testing the LoopClosureDetector without running the entire VIO
pipeline.

The package requires Spark VIO, which in turn has several other dependencies
including gtsam, OpenCV, and OpenGv.

Currently only online rosbag parsing is available, but an offline implementation
is coming soon.

## Dependencies

Install [Spark VIO](https://github.mit.edu/SPARK/VIO) and all its dependencies
by following the install instructions found in that repo.

**NOTE:** You will need a version of Spark VIO that has the LoopClosureDetector.
Currently, the best branch to use is [`feature/loop_closure`](https://github.mit.edu/SPARK/VIO/tree/feature/loop_closure)

## Features

The [`LCD_ros_node`](/src/LCD_ros_node.cpp) is the main node that spins the
LoopClosureDetector pipeline. It expects stereo images published to two
topics with message type `sensor_msgs::Image`. It will store these images in
a local database and detect loop closures between similar images.

The node outputs an image including both frames involved in the loop closure
as well as the feature correspondences drawn between the frames. All features
are visualized as blue circles, while the correspondences selected to perform
pose recovery have lines drawn between them.

The node also outputs a `geometry_msgs::PoseArray` and
`visualization_msgs::MarkerArray` type message that can be visualized in RVIZ
if ground truth pose information is available. The ground-truth reference frame
pose for each loop closure is sent to RVIZ, along with the relative pose of
the current frame for each loop closure and the ground-truth pose of that
current frame. These pose are all visualized as yellow arrows.

The `MarkerArray` includes green arrows showing the connection between the
reference frame and the computed current frame pose. The red arrows connect
that computed current frame pose with the ground-truth current frame, as a
quick means to visualize error.

In general, large red arrows means high error in the pose recovery.

## Usage

To run with one of the EuRoC datasets, simply run
```
roslaunch lcd_ros LCD_ros_euroc.launch
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
Change the `body_frame` argument accordingly. However, for datasets like
EuRoC which send the transform to a normal rostopic instead, change the
`gnd_truth_topic` and include the
[`transform_converter_node`](/scripts/transform_converter_node.py) as in the
example launch file. This node simply publishes the transforms from the topic
to the `/tf` tree.

In addition, parameters for the camera must be defined. Make a new folder
in [`/param`](/param) entitled with the name of your dataset and include
a [`calibration.yaml`](/param/EUROC/calibration.yaml) file detailing the
camera parameters, as well as a [`trackerParameters.yaml`](/param/EUROC/trackerParameters.yaml) file.

You can then change the dataset name in your launch file and the redirection
will be done automatically. In this parameter folder, you can also define
custom parameters for the LoopClosureDetector module.

## ROS Launch Arguments

* `bool log_output` is sent to LoopClosureDetector for sending output to a log file
* `int verbosity` is sent to LoopClosureDetector for printing messages to console
* `bool visualize` is for starting and publishing to RVIZ

## TODO

-[ ] Support for offline rosbag parsing
-[ ] Publish numerical error in loop closures based on ground truth
-[ ] Update ROS wrapper to current spark_vio_ros framework
-[ ] Refactor to inherit from spark_vio_ros to prevent code duplication
-[ ] CMakeLists.txt and package.xml cleanup
-[ ] Get rid of trackerParameters.yaml requirement
