#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <iostream>
#include <string>


nav_msgs::Odometry msg;
bool is_init;

tf2_ros::Buffer tf_buffer;
ros::Publisher gmappingOdomPub;
using namespace std;
string ns;

void odomcb(const nav_msgs::Odometry::ConstPtr& msg){
    geometry_msgs::TransformStamped odom_to_map = tf_buffer.lookupTransform("map", ns + "/odom", ros::Time(0), ros::Duration(1.0));
    nav_msgs::Odometry gmappingOdom;
    gmappingOdom.header.frame_id="map";
    gmappingOdom.header.stamp=ros::Time::now();
    tf2::doTransform(msg->pose.pose, gmappingOdom.pose.pose, odom_to_map);
    gmappingOdomPub.publish(gmappingOdom);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "publish_pf_pose");
    ros::NodeHandle nh;
    nh.getParam("ns", ns);
    tf2_ros::TransformListener tf2_listener(tf_buffer);

    ros::Subscriber odomSub = nh.subscribe<nav_msgs::Odometry>("/" + ns + "/odometry/filtered", 1, &odomcb);
    gmappingOdomPub = nh.advertise<nav_msgs::Odometry>("/" + ns + "/gmapping/odometry", 10);

    ros::spin();
    return 0;
}