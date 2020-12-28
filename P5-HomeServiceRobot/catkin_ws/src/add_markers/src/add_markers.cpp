#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

// -------- Change goal here -
// Goal 1
double GOAL1_X = -3.5, GOAL1_Y = -2.75;
// Goal 2
double GOAL2_X = 6.5, GOAL2_Y = -5.2;
// ---------

bool gObjectPicked = false;
bool gObjectDropped = false;

void addMarker(ros::Publisher& pub, visualization_msgs::Marker& marker, const float x, const float y)
{
    ROS_INFO("marker ADDing at x=%0.2f, y=%0.2f", x, y);
    marker.header.stamp = ros::Time::now();
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.action = visualization_msgs::Marker::ADD;
    // publish
    pub.publish(marker);
}

void deleteMarker(const ros::Publisher& pub, visualization_msgs::Marker& marker)
{
    ROS_INFO("marker DELETING at x=%0.2f, y=%0.2f", marker.pose.position.x, marker.pose.position.y);

    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::DELETE;
    // publish
    pub.publish(marker);
}

// TRUE if robot (rx,ry) is in given goal (gx, gy) zone
bool withinGoalZone(double rx, double ry, double gx, double gy)
{
    double threshold = 0.5;

    // RMS distance of robot (rx,ry) to goal(gx,gy)
    double distance = sqrt( pow((rx-gx), 2) + pow((ry-gy), 2) ) ;

    ROS_INFO("  [distance to goal: %0.2f]", distance);

    if (distance <= threshold)
        return true;
    else
        return false;

}

// handle callback - when Pick Obj informs us Goal is reached
void handleOdometryMessage(const nav_msgs::Odometry::ConstPtr& msg)
{
    float robot_x = msg->pose.pose.position.x;
    float robot_y = msg->pose.pose.position.y;

    if (!gObjectPicked && !gObjectDropped) {
        if (withinGoalZone(robot_x, robot_y, GOAL1_X, GOAL1_Y)) {
            ROS_INFO("  Robot in pickup zone");
            gObjectPicked = true;
        }
    }

    if (gObjectPicked && !gObjectDropped) {
        if (withinGoalZone(robot_x, robot_y, GOAL2_X, GOAL2_Y)) {
            ROS_INFO("  Robot in drop-off zone");
            gObjectDropped = true;
        }
    }

}

// called when Publisher (pick objects) informs we reached goals
void handleGoalMessage(const std_msgs::String msg) 
{
    ROS_INFO_STREAM("  handleGoalMsg called with data: " << msg.data << "\n");
    if (msg.data == "PICKED_UP") {
        ROS_INFO(" >> Robot picked up Goal #1");
        gObjectPicked = true;
        // directly delete marker?

    } else if (msg.data == "DROPPED_OFF") {
        ROS_INFO(" >> Robot reached Goal #2");
        gObjectDropped = true;
        // directly add marker?
    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "add_markers");

    ros::NodeHandle n;

    // publisher for Marker 
    ros::Publisher marker_pub;
    // our marker object
    visualization_msgs::Marker marker;

    ROS_INFO("add_markers: active");


    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // # Robot position subscriber
    // ros::Subscriber position_sub = n.subscribe("/odom", 1000, handleOdometryMessage);
    ros::Subscriber goal_subscriber = n.subscribe("/goal_action", 1, handleGoalMessage); 

    // shape
    uint32_t shape = visualization_msgs::Marker::CUBE;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type.  
    marker.type = shape;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();  // keep until deleted
    ros::Duration(1.0).sleep();

//    // 1 - Add marker at Goal 1
    addMarker(marker_pub, marker, GOAL1_X, GOAL1_Y);
    ros::Duration(2.0).sleep();

//    // 2 - Delete marker at Goal 1
//    deleteMarker(marker_pub, marker);
//    ros::Duration(5.0).sleep();

//    // 3 - Add marker at Goal 2
//    addMarker(marker_pub, marker, GOAL2_X, GOAL2_Y);
//    ros::Duration(5.0).sleep();
//

    while(ros::ok()) {

        if (gObjectPicked) {
            deleteMarker(marker_pub, marker);
            // ros::Duration(5.0).sleep();
            gObjectPicked = false; // to remove loop
        }

        if (gObjectDropped) {
            addMarker(marker_pub, marker, GOAL2_X, GOAL2_Y);
            gObjectDropped = false; // to remove loop
            ros::Duration(2.0).sleep();
            break;
        }

        ros::spinOnce();

    } // while

    return 0;

}
