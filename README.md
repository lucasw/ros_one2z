# ros1, ros2, zenoh

Similar node graphs in vanilla/out-of-the-box ros1 and ros2 compared (except that ros1 is `one` version running on the latest Ubuntu, ros2 is `iron` or perhaps `rolling` on same OS), focus on localhost using python.  Also serializing messages into zenoh.

* Publish mupltiple topics of ~2000x1000 color images at 30Hz, make something moving in them
* have a pipeline of 3 or 4 nodes that do operations on the images and publish a modified versions, or process pixels and publish a smaller message as a result- perhaps mask out a color or brightness level mask in the first step, then publish a simplified contour of the masked region in the next node
* View multiple images in rqt
* Publish a bunch of transforms to /tf, have some tips of the tf tree correspond to image frame_ids
* View something in 3D in rviz, also 1 or 2 camera overlay plugins running
* rosbag record subset of images, compressed or uncompressed

Later publish and process a point cloud.

Probably won't devote too much to this until Jazzy and the alternative middleware (https://discourse.ros.org/t/ros-2-alternative-middleware-report/33771) arrives (or Rolling will have it earlier?), but it will be nice to have something to run against it when it does arrive.

## Setup

The launch files below don't yet exist, but this is the idead of what they ought to be.

### ros1

Follow https://github.com/lucasw/ros_from_src/blob/debianize/ubuntu_2204/README.md to build ros `one` on Ubuntu 23.04.

Then make an overlaying workspace:

```
mkdir -p ~/ros/ros1_one/src
cd ~/ros/ros1_one/src
git clone git@github.com:lucasw/ros1vs2.git
cd ..
source  ~/install_base_catkin_ws/install/setup.bash
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -Wno-deprecated
catkin build
source devel/setup.bash
ros2 launch ros1vs2 ros1vs2.launch
```


### ros2

Follow https://github.com/lucasw/ros_from_src/issues/29 to build ros2 `iron`

Then make an overlaying workspace

```
mkdir -p ~/ros/ros2_iron_misc/src
cd ~/ros/ros2_iron_misc/src
ln -s ../../ros1_one/src/ros1vs2
cd ..
source ~/ros/ros2_iron/install/local_setup.bash
colcon build --symlink-install --packages-skip ros1_example_pkg
source install/local_setup.bash
ros2 launch ros1vs2 ros1vs2.launch
```

### zenoh

#### install

```
cd ~/other/src/zenoh
git clone git@github.com:eclipse-zenoh/zenoh-c.git
cd ~/other/build
mkdir build/zenoh-c
cd zenoh-c
cmake ../../src/zenoh/zenoh-c -DCMAKE_INSTALL_PREFIX=$DEST -DCMAKE_BUILD_TYPE=Release
cmake --build . --target install
```

```
cd ~/other/src/zenoh
git clone git@github.com:eclipse-zenoh/zenoh-cpp.git
cd ~/other/build
mkdir zenoh-cpp
cd zenoh-cpp
cmake ../../src/zenoh/zenoh-cpp/install -DCMAKE_INSTALL_PREFIX=$DEST -DCMAKE_BUILD_TYPE=Release
make
make install
```

Then when catkin building:

```
catkin config --cmake-args -DCMAKE_CXX_FLAGS=-I$DEST/include
catkin build one2z ros1_example_pkg
```

#### examples

Instead of using https://github.com/eclipse-zenoh/zenoh-plugin-ros1 have ros messages published directly into zenoh using serialization and deserialization.

```
rosrun ros1_example_pkg zenoh_generate_image.py
rosrun ros1_example_pkg zenoh_image_to_contour.py
```

The ros parameter server and roscore is still used, but the actual messages go through zenoh.

# ros1 mcap direct recording and playback

(inside a venv in Ubuntu 23.04 and later)

```
pip install mcap mcap-ros1-support
```

Create a test mcap (in C++):
```
rosrun one2z mcap_recorder
```

(it will be adapted to record from a single topic of any type later, currently it creates Float64 messages internally)


Play back an mcap (should work with any with ros1 messages in it):
```
rosrun one2z ros1_play_mcap.py test.mcap
```
