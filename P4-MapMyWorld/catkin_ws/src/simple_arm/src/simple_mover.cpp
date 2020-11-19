#include "ros/ros.h"
#include "std_msgs/Float64.h"

int main(int argc, char **argv)
{
	// init arm_mover node
	ros::init(argc, argv, "arm_mover");

	// Create handle to arm_mover
	ros::NodeHandle n;

	// Create a publisher to publish to joint 1i
	ros::Publisher joint1_pub = n.advertise<std_msgs::Float64>("/simple_arm/joint_1_position_controller/command", 10);
	// Create a publisher to publish to joint 2
	ros::Publisher joint2_pub = n.advertise<std_msgs::Float64>("/simple_arm/joint_2_position_controller/command", 10);

	// Set loop frequency to 10 Hz
	ros::Rate loop_rate(10);

	int start_time, elapsed;

	// Get ROS start time
	while (not start_time) {
		start_time = ros::Time::now().toSec();
	}

	// ROS loop
	while (ros::ok()) {
		// get ROS elapsed time
		elapsed = ros::Time::now().toSec() - start_time;

		// Set the arm joint anglesi
		std_msgs::Float64 joint1_angle, joint2_angle;
		joint1_angle.data = sin(2 * M_PI * 0.1 * elapsed) * (M_PI / 2);
		joint2_angle.data = sin(2 * M_PI * 0.1 * elapsed) * (M_PI / 2);

		// Publish the messages
		joint1_pub.publish(joint1_angle);
		joint2_pub.publish(joint2_angle);

		// sleep till 10 hz reached
		loop_rate.sleep();
	}

	return 0;

} // main
