#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>

void addMarker(ros::Publisher& pub, visualization_msgs::Marker& marker, const float x, const float y)
{
    ROS_INFO("marker ADDing at x=%0.2f, y=%0.2f", x, y);
    marker.header.stamp = ros::Time::now();
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.action = visualization_msgs::Marker::ADD;
    // publish
    pub.publish(marker);
    // TODO - inform any subscribers

}
void deleteMarker(const ros::Publisher& pub, visualization_msgs::Marker& marker)
{
    ROS_INFO("marker DELETING at x=%0.2f, y=%0.2f", marker.pose.position.x, marker.pose.position.y);

    marker.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::DELETE;
    // publish
    pub.publish(marker);
    // TODO - inform any subscribers

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

    // -------- Change goal here -
    // Goal 1
    float goal1_x = -3.5, goal1_y = -2.75;
    // Goal 2
    float goal2_x = 6.5, goal2_y = -5.2;
    // ---------

    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

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

    // 1 - Add marker at Goal 1
    addMarker(marker_pub, marker, goal1_x, goal1_y);
    ros::Duration(5.0).sleep();

    // 2 - Delete marker at Goal 1
    deleteMarker(marker_pub, marker);
    ros::Duration(5.0).sleep();

    // 3 - Add marker at Goal 2
    addMarker(marker_pub, marker, goal2_x, goal2_y);
    ros::Duration(5.0).sleep();

    // ros::spin();
    return 0;

}
