#!/bin/bash

# below are the packages that are require to be installed in order to run correctly
VELODYNE_PKG="velodyne"
POINTCLOUD_TO_LASERSCAN_PKG="pointcloud_to_laserscan"
HECTOR_SLAM_PKG="hector_slam"
NAVIGATION_PKG="costmap_2d"
AMCL_PKG="amcl"
MOVE_BASE_PKG="move_base"
LASER_SCAN_MATCHER_PKG="laser_scan_matcher"
ACKERMANN_STEERING_LOCAL_PLANNER="teb_local_planner"
SBG_DRIVER="sbg_driver"
KEY_TELEOP="key_teleop"

# this script allows functionality to run with -r in order to reinstall libraries
# even if a grep returns that they are in the system

case $1 in
    -r)
        sudo apt-get install ros-kinetic-velodyne
        sudo apt-get install ros-kinetic-pointcloud-to-laserscan
        sudo apt-get install ros-kinetic-hector-slam
        sudo apt-get install ros-kinetic-navigation
        sudo apt-get install ros-kinetic-amcl
        sudo apt-get install ros-kinetic-move-base
        sudo apt-get install ros-kinetic-laser-scan-matcher
        sudo apt-get install ros-kinetic-teb-local-planner
        sudo apt-get install ros-kinetic-sbg-driver
	sudo apt-get install ros-kinetic-key-teleop
esac

# the rest will run if there are no arguments supplied to the script

if [ -z $1 ]
then

    # installs the velodyne drivers if not found on the system

    if ! rospack list-names | grep -q $VELODYNE_PKG
    then
        echo "Installing ${VELODYNE_PKG} package."
        sudo apt-get install ros-kinetic-velodyne
    else
        echo "${VELODYNE_PKG} already found on system."
    fi

    # installs the laser scan to point cloud package if not found on the system

    if ! rospack list-names | grep -q $POINTCLOUD_TO_LASERSCAN_PKG
    then
        echo "Installing ${POINTCLOUD_TO_LASERSCAN_PKG} package."
        sudo apt-get install ros-kinetic-pointcloud-to-laserscan
    else
        echo "${POINTCLOUD_TO_LASERSCAN_PKG} already found on system."
    fi

    # installs the hector slam package if not found on the system

    if ! rospack list-names | grep -q $HECTOR_SLAM_PKG
    then
        echo "Installing ${HECTOR_SLAM_PKG} package."
        sudo apt-get install ros-kinetic-hector-slam
    else
        echo "${HECTOR_SLAM_PKG} already found on system."
    fi

    # install the navigation package if not found on the system

    if ! rospack list-names | grep -q $NAVIGATION_PKG
    then
        echo "Installing ${NAVIGATION_PKG} package."
        sudo apt-get install ros-kinetic-navigation
    else
        echo "${NAVIGATION_PKG} already found on system."
    fi

    if ! rospack list-names | grep -q $AMCL_PKG
    then
        echo "Installing ${AMCL_PKG} package."
        sudo apt-get install ros-kinetic-amcl
    else
        echo "${AMCL_PKG} already found on system."
    fi

    if ! rospack list-names | grep -q $MOVE_BASE_PKG
    then
        echo "Installing ${MOVE_BASE_PKG} package."
        sudo apt-get install ros-kinetic-move-base
    else
        echo "${MOVE_BASE_PKG} already found on system."
    fi

    if ! rospack list-names | grep -q $LASER_SCAN_MATCHER_PKG
    then
        echo "Installing ${LASER_SCAN_MATCHER_PKG} package."
        sudo apt-get install ros-kinetic-laser-scan-matcher
    else
        echo "${LASER_SCAN_MATCHER_PKG} already found on system."
    fi

    if ! rospack list-names | grep -q $ACKERMANN_STEERING_LOCAL_PLANNER
    then
        echo "Installing ${ACKERMANN_STEERING_LOCAL_PLANNER} package"
        sudo apt-get install ros-kinetic-teb-local-planner
    else
        echo "${ACKERMANN_STEERING_LOCAL_PLANNER} already found on system."
    fi

    if ! rospack list-names | grep -q $SBG_DRIVER
    then
        echo "Installing ${SBG_DRIVER} package"
        sudo apt-get install ros-kinetic-sbg-driver
    else
        echo "${SBG_DRIVER} already found on system."
    fi
    
    if ! rospack list-names | grep -q $KEY_TELEOP
    then
	echo "Installing ${KEY_TELEOP} package"
	sudo apt-get install ros-kinetic-key-teleop
    else
	echo "${KEY_TELEOP} already found on system."
    fi

fi



