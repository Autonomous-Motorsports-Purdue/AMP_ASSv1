# AMP-v1
AMP Software and Electrical Stack for the 1st iteration of the kart. This version is focused on racing at the EVGrandPrix Autonomous Race (barriers on either side of the track).

Learn more about software v1 [here](https://drive.google.com/file/d/1K5XBHzRQoebuGRryKY5umeLOhvQEEuj8/view?usp=sharing).

## Installing Dependencies
To install all necessary dependencies, run the following command at the top of the directory (in AMP-v1):
```
rosdep install --from-paths src --ignore-src -r -y
```
To learn more about [rosdep](http://wiki.ros.org/rosdep), check out the [ROS wiki](http://wiki.ros.org/rosdep).

## Building
As mentioned in the [ROS core tutorials on building packages](http://wiki.ros.org/ROS/Tutorials/BuildingPackages), run the following two commands at the top of the directory (in AMP-v1) to build each of the ROS packages in our project.
```
catkin_make
source devel/setup.bash
```
