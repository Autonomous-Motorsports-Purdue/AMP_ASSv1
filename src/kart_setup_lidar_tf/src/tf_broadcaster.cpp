#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

/* Base Link to Base Laser Contstants (meters) */
#define BLNK_BLZR_DX 1.1800
#define BLNK_BLZR_DY 0.0000
#define BLNK_BLZR_DZ 0.3048

int main(int argc, char ** argv){
	ros::init(argc, argv, "robot_tf_publisher");
	ros::NodeHandle n;

	ros::Rate r(100)

	tf::TransformBroadcaster broadcaster;
	tf::Quaternion quat = tf::Quaternion(0, 0, 0, 1);
	tf::Vector3 blnk_blzr = tf:Vector3(BLNK_BLZR_DX, BLNK_BLZR_DY, BLNK_BLZR_DZ);

	while(n.ok()){
		broadcaster.sendTransform(tf::StampedTransform(tf::Transform(quat, blnk_blzr),
					  		       ros::Time::now(), 
					  		       "base_link", 
					  		       "base_laser"));
		r.sleep();
	}
}
