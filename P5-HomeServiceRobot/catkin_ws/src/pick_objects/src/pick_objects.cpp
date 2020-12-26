#include <ros/ros.h>
#include <std_msgs/String.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){

  int rc = 0; // return code

  double X_PICK = -3.5, Y_PICK = -2.75;  // Pickup 
  double X_DROP = 6.5, Y_DROP = -5.2; // Drop off location
  // double X_DROP = 7.5, Y_DROP = -1.2; // Drop off location

  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  ros::NodeHandle n;

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Publisher to inform when reached Goals
  // "goal_action" == "PICKED_UP" or "DROPPED_OFF"
  ros::Publisher goal_pub = n.advertise<std_msgs::String>("/goal_action", 1);
  // message
  std_msgs::String goal_msg;
  std::string msg_info;

  ROS_INFO_STREAM("pick_objects started...");

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // wait till there's at least 1 subscriber for goal_action
  while(goal_pub.getNumSubscribers() < 1) {
      if (!ros::ok()) {
          ROS_INFO(" >> some ROS error. exiting!");
          return -1;
      }
      ROS_WARN_ONCE("Please start add_markers or create atleast 1 subscriber for /goal_action!");
      sleep(1);
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
    ROS_INFO("Hooray, SUCCESS with Goal #1 - pickup!");

    // inform subscribers that we picked up Goal 1
    msg_info = "PICKED_UP";
    goal_msg.data = msg_info;
    goal_pub.publish(goal_msg);

    // sleep for 5s to sim pickup
    ROS_INFO(" - sleeping 5s - ");
    ros::Duration(5).sleep();

    // Goal #2 - Drop off
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = X_DROP;
    goal.target_pose.pose.position.y = Y_DROP;
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("[Sending drop-off goal to robot]");
    ac.sendGoal(goal);

    ROS_INFO_STREAM("Robot now traveling to goal #2: drop-off at x: " << X_DROP << " y: " << Y_DROP);
    ac.waitForResult(); // wait infinite

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Yippee! Robot reached drop-off zone!");
      msg_info = "DROPPED_OFF";
      goal_msg.data = msg_info;
      goal_pub.publish(goal_msg);   // inform subscriber we reached goal #2
      ros::Duration(5).sleep();
    } else {
      ROS_INFO_STREAM("Dang! Robot failed drop off. >>> State: " << ac.getState().toString().c_str());
      rc = 2;
    }
  }
  else {
    ROS_INFO_STREAM("Oops! Robot failed to get to pickup location. >>>State: " << ac.getState().toString().c_str());
    rc = 1;
  }

  ROS_INFO_STREAM("Robot exiting with code: " << rc);
  return rc;
}
