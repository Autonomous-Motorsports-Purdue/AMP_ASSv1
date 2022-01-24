#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <cstdlib>

int main(int argc, char **argv) {
  ros::init(argc, argv, "hector_clear_map");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<std_srvs::Trigger>("/hector_mapping/reset_map");
  std_srvs::Trigger srv;

  if (ros::service::waitForService("/hector_mapping/reset_map", /*timeout=*/10000/*10 sec*/)) {
    ROS_INFO("Found service /hector_mapping/reset_map");
  } else {
    ROS_ERROR("Failed to find service /hector_mapping/reset_map.");
    return 1;
  }

  ros::Rate r(1); // period of 1 sec
  while (ros::ok()) {
    if (client.call(srv)) {
      if (srv.response.success) ROS_INFO("Successfully cleared map");
      else ROS_ERROR("Service /hector_mapping/reset_map responded with failure.");
    } else {
      ROS_ERROR("Failed to call service /hector_mapping/reset_map.");
      return 1;
    }
    r.sleep();
  }

  return 0;
}
