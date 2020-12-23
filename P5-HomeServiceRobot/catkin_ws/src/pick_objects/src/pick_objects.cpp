#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){

  // TODO: Create x,y coords
  double X_PICK = -3.5, Y_PICK = -2.75;  // Pickup 
  double X_DROP = 6.5, Y_DROP = -5.2; // Drop off location
  // double X_DROP = 7.5, Y_DROP = -1.2; // Drop off location

  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = X_PICK;
  goal.target_pose.pose.position.y = Y_PICK;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO_STREAM("Sending goal #1: pickup at x: " << X_PICK << " y: " << Y_PICK);
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Hooray, SUCCESS with Goal #1 - pickup!\n Robot now waiting for 5s...");

    // sleep for 5s to sim pickup
    ros::Duration(5).sleep();

    // Goal #2 - Drop off
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = X_DROP;
    goal.target_pose.pose.position.y = Y_DROP;
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("[Sending drop-off goal]");
    ROS_INFO_STREAM("Robot now traveling to goal #2: drop-off at x: " << X_DROP << " y: " << Y_DROP);

    ac.sendGoal(goal);
    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Yippee! Robot reached drop-off zone!");
    }
    else {
      ROS_INFO("Dang! Robot did not reach drop-off zone :-(");
    }
  }
  else {
    ROS_INFO("Oops! Robot failed to get to pickup location :-( ");
  }

  ROS_INFO("Robot exiting...");
  return 0;
}
