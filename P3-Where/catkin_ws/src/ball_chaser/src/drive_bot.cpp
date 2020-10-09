#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;

// Create a handle_drive_request callback function that executes whenever a drive_bot service is requested
// This function should publish the requested linear x and angular velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities
bool drive_request_handler(ball_chaser::DriveToTarget::Request& req,
                            ball_chaser::DriveToTarget::Response& res)
{
    ROS_INFO("drive_bot: DriveToTarget - received: linear_x: %1.2f, angular_z: %1.2f",
            (float) req.linear_x, (float) req.angular_z);

    // 1 - Publish command velocity to /cmd_vel
    geometry_msgs::Twist motor_command;

    motor_command.linear.x = req.linear_x; 
    motor_command.angular.z = req.angular_z;

    // 2 - publish 
    motor_command_publisher.publish(motor_command);

    // 3 - response msg
    res.msg_feedback = "Velocity cmd sent: linear_x=" + std::to_string(motor_command.linear.x) + 
        " angular_z=" + std::to_string(motor_command.angular.z);

    ROS_INFO_STREAM(res.msg_feedback);

    return true;
}

int main(int argc, char** argv)
{
    // 1. Initialize a ROS node
    ros::init(argc, argv, "drive_bot");
    ROS_INFO_STREAM("Initializing drive_bot node...");

    // 2. Create a ROS NodeHandle object
    ros::NodeHandle n;

    // 3. Inform ROS master that we will be publishing a message of type 
    // geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // 4. Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", drive_request_handler);
    ROS_INFO_STREAM("drive_bot: ready to send command velocities to robot..."); 

    // 5. Handle ROS communication events
    ros::spin();

    return 0;
}

