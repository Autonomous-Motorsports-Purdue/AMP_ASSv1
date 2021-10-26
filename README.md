# AMP-v1
The Autonomous Software System for the 1st iteration of the AMP autonomous go-kart. This version is focused on racing at the EVGrandPrix Autonomous Race (barriers on either side of the track).

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

## Running
The preferred method for running the system in its various configurations si `roslaunch`ing with launch files located in `src/kart_2dnav/launch/`. 

There are two main types of launch files:
* Simulation: Launch files that start with `sim_` correspond to launching a configuration of our system in an RViz simulation.
* Physical: Launch files that start with `kart_` correspond to launching a configuration of our system that is meant for physical testing. 

All other launch files are either child launch files (i.e. used by other the `sim` and `kart` launch files to launch different sub-systems) or are archived.

For example, to run the simulation of the kart on the autocross map/track the command (after successfully building) is 
```
roslaunch kart_2dnav sim_autocross_track.launch
```

See the individual launch files in  `src/kart_2dnav/launch/` for more info on the purpose/configuration of that launch file.

## Troubleshooting
A secion outlining common issues and their fixes.
* If an error is encountered claiming that a number of packages were not found, or that modules are not present. Try updating all dependencies in the Kinetic ROS installation with:
```
rosdep update --include-eol-distros
```
