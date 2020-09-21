#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // 1: Request a service and pass the velocities to it to drive the robot
    ROS_INFO_STREAM("img_proc::drive_robot: requesting to move robot");

    ball_chaser::DriveToTarget service;

    // set linear and angular velocities
    service.request.linear_x = lin_x;
    service.request.angular_z = ang_z;

    // 2 - request the service /ball_chaser/command_robot with given velocities
    if (!client.call(service)) {
        ROS_ERROR("img_processor: Failed to request robot to move /command_robot!");
    }

}


// Callback method - processes image to find where the white ball is located
// Instructs the robot to drive in the general direction of the white ball (if any)
void process_image_callback(const sensor_msgs::Image img)
{
    // Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    const int WHITE = 255;

    int white_count = 0;                // num of white pixels
    int white_center = -1;              // center of all white pixels
    int white_x = -1;
    int white_x_sum = 0;        
    float lin_x = 0.0, ang_z = 0.0;     // velocities for robot

    // 1 - Loop thru image to see if any white pixels
    for (int i = 0; i < img.height * img.step - 2; i=i+3) {
        // if white pix - in all R, G, B channels
        if ((img.data[i] == WHITE) && 
            (img.data[i+1] == WHITE) && 
            (img.data[i+2] == WHITE)) {
            // get x pos of white
            white_x =  i % img.step;
            white_count++;
            white_x_sum += white_x; // sum up all x positions
        }
    }


    if (white_count > 0) {
        // 2a - find center avg of all white x positions
        white_center = white_x_sum / white_count;  
        // remove DEBUG
        // ROS_INFO("img H %d, W: %d, white count: %d, ball center: %d", 
        //     img.height, img.step, white_count, white_center);

        const float MAX_ANG_Z = 0.5;            // max ang_z velocity  
        const int IMG_THIRD = img.step / 3;     // # pixels in 1/3 of img width
        const int TH_LEFT = img.step / 3;       // our left threshold
        const int TH_RIGHT = img.step * 2 / 3;  // our right threshold

        // 2b - see where the center of white pixels
        if (white_center < TH_LEFT) {
            // how far left?
            int diff = TH_LEFT - white_center; // NOTE: this should be +ve
            // scale the ang velocity by how far left
            ang_z = MAX_ANG_Z * diff / IMG_THIRD;
            lin_x = 0.3;
            // debug
            ROS_DEBUG("white count: %d, ball center: %d, diff: %d, ang %1.2f", 
             white_count, white_center, diff, ang_z);
        } else if (white_center > TH_RIGHT) {
            // how far right?
            int diff = TH_RIGHT - white_center; // NOTE: this is negative
            // scale the ang velocity by how far right
            ang_z = MAX_ANG_Z * diff / IMG_THIRD;
            lin_x = 0.3;
            // debug
            ROS_DEBUG("white count: %d, ball center: %d, diff: %d, ang %1.2f", 
             white_count, white_center, diff, ang_z);
        } else {
            // FORWARD
            ang_z = 0.0;
            lin_x = 0.25;
        }
    } else {
        // STOP
        lin_x = 0.0;
        ang_z = 0.0;
    }

    // 3 - Drive the robot
    ROS_INFO("image_processor: sending : lin_x: %1.2f, ang_z: %1.2f", lin_x, ang_z);
    drive_robot(lin_x, ang_z);

}


int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;
    ROS_INFO_STREAM("image processor node started...");

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);
    ROS_INFO_STREAM("image_processor: subscribed to camera - waiting to chase white ball!");

    // Handle ROS communication events
    ros::spin();

    return 0;
}
