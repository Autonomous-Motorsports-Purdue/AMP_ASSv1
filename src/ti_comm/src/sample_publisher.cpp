#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "sample_cmd_vel_publisher");
  ros::NodeHandle handle;

  ros::Publisher sample_publisher = handle.advertise<geometry_msgs::Twist>("sample_cmd_vel", 100);
  ros::Rate loop_rate(10);
  while(ros::ok()) {
    geometry_msgs::Twist msg;
    msg.linear.x = 1.0;
    msg.linear.y = 5.2;
    msg.linear.z = 0;

    msg.angular.x = 3.24;
    msg.angular.y = 9;
    msg.angular.z = 0.0;

    ROS_INFO("Publisher is publishing stuff");
    sample_publisher.publish(msg);
    // ros::spinOnce();
    loop_rate.sleep();
  }
  return EXIT_SUCCESS;
}
