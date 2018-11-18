#include <ros/console.h>
#include "ros/ros.h"
#include "gnss_l86_interface/GnssData.h"
#include "cartesian_pose/CartesianPose.h"
#include <gnss_l86_interface/gnss_l86_lib.h>

position last_position;

void gnss_data_callback(const gnss_l86_interface::GnssData::ConstPtr& gnss_msg)
{
    last_position.latitude = gnss_msg->latitude;
    ROS_INFO(" ");
    ROS_INFO("-----------------------");
    ROS_INFO_STREAM("Latitude: " << gnss_msg->latitude);
    ROS_INFO_STREAM("Last Position Latitude: " << last_position.latitude);
    ROS_INFO_STREAM("Longitude: " << gnss_msg->longitude);
    ROS_INFO_STREAM("Timestamp: " << gnss_msg->timestamp);
    ROS_INFO("-----------------------");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cartesian_pose_node");
    ros::NodeHandle n;
    ros::Publisher publisher = n.advertise<cartesian_pose::CartesianPose>("cartesian_pose", 1000);
    ros::Subscriber subscriber = n.subscribe("gnss_data", 1000, gnss_data_callback);
    ros::Rate loop_rate(0.5);
    cartesian_pose::CartesianPose pose;

    while (ros::ok())
    {
        pose.is_estimated = true;
        pose.x = 1;
        pose.y = 1;
        pose.z = 1;
        pose.bearing = 27;
        pose.timestamp = 1234;

        publisher.publish(pose);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}