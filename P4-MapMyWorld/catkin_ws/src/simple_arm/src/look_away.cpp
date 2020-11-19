#include "ros/ros.h"
#include "simple_arm/GoToPosition.h"
// for JointState
#include <sensor_msgs/JointState.h>
// for camera data
#include <sensor_msgs/Image.h>


class SubscribeAndPublish
{
  // state of arm: moving or not
  bool moving_state_ = false;
  // last position of joints
  std::vector<double> last_position_{ 0, 0 };

  ros::ServiceClient client_; 
  ros::NodeHandle n_; 
  // ros::Publisher pub_;
  ros::Subscriber sub1_;
  ros::Subscriber sub2_;

  // Helper func to move the arm to center
  // calls 'safe_move' service
  void move_arm_center()
  {
      ROS_INFO_STREAM("Moving the arm to the center...");

      // Request centered joint angles [1.57, 1.57] (meaning PI/2 each)
      simple_arm::GoToPosition simpleArmService;
      simpleArmService.request.joint_1 = 1.57;
      simpleArmService.request.joint_2 = 1.57;

      // call the 'safe_move' service and pass the requested joint angles
      if (!client_.call(simpleArmService))
          ROS_ERROR("Failed to call service safe_move");
  }

public:
  SubscribeAndPublish()
  {
    ROS_INFO_STREAM("Initializing LookAway node...");
    // create and save client obj
    client_ = n_.serviceClient<simple_arm::GoToPosition>("/arm_mover/safe_move");

    //Topic you want to publish
    // pub_ = n_.advertise<PUBLISHED_MESSAGE_TYPE>("/published_topic", 1);

    // Subscriber to /simple_arm/joint_states to read arm state
    sub1_ = n_.subscribe("/simple_arm/joint_states", 10, 
                &SubscribeAndPublish::handle_joint_state, this);
    // Subscriber to camera image
    sub2_ = n_.subscribe("/rgb_camera/image_raw", 10, 
                &SubscribeAndPublish::handle_image_raw_look_away, this);

    ROS_INFO_STREAM("... LookAway node init'ed");
  }

  // Handler - for raw image: 
  // If image is uniform, then look away
  void handle_image_raw_look_away(const sensor_msgs::Image& img)
  {
      bool uniform_image = true;

      // Loop thru each pixel, and check if it's uniform (same)
      for (int i = 0; i < img.height * img.step; i++) {
          if (img.data[i] - img.data[0] != 0) {
                  uniform_image = false;
                  break;
          }
      }
      // if image is uniform, and arm is not moving, move it to the center
      if (uniform_image == true && moving_state_ == false) {
        move_arm_center();
      }
  }

  // Handler - for joint states
  // Saves the position of the arms
  void handle_joint_state(const sensor_msgs::JointState& js)
  {
        // Get joint states current pos
        std::vector<double> current_position = js.position;

        // define tolerance threshold to compare double values
        double tolerance = 0.0005;

        // check if arm is moving
        // by comparing current position to given position
        if (fabs(current_position[0] - last_position_[0]) < tolerance 
                && fabs(current_position[1] - last_position_[1]) < tolerance) {
            moving_state_ = false;
        } else {
            // save the joints state
            moving_state_ = true;
            last_position_ = current_position;

        }
  }


};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS and look_away node
  ros::init(argc, argv, "look_away");
  ROS_INFO_STREAM("..... initializing look_away node.....");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish  SAPObject;

  ros::spin();

  return 0;
}

