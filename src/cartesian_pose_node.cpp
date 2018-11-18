#include <ros/console.h>
#include "ros/ros.h"
#include "gnss_l86_interface/GnssData.h"
#include "cartesian_pose/CartesianPose.h"
#include "gnss_l86_interface/gnss_l86_lib.h"

position gnss_position;
bool new_gnss = false;

void gnss_data_callback(const gnss_l86_interface::GnssData::ConstPtr& gnss_msg)
{
    gnss_position.latitude = gnss_msg->latitude;
    gnss_position.longitude = gnss_msg->longitude;
    gnss_position.fix = gnss_msg->fix;
    gnss_position.number_of_satelites = gnss_msg->number_of_satelites;
    gnss_position.horizontal_precision = gnss_msg->horizontal_precision;
    gnss_position.altitude = gnss_msg->altitude;
    gnss_position.timestamp = gnss_msg->timestamp;
    new_gnss = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cartesian_pose_node");
    ros::NodeHandle n;
    ros::Publisher publisher = n.advertise<cartesian_pose::CartesianPose>("cartesian_pose", 1000);
    ros::Subscriber subscriber = n.subscribe("gnss_data", 1000, gnss_data_callback);
    ros::Rate loop_rate(1000);
    cartesian_pose::CartesianPose pose;

    while (ros::ok())
    {
        if (new_gnss)
        {
            ROS_INFO(" ");
            ROS_INFO("--------- NEW GNSS ---------");
            ROS_INFO_STREAM("Latitude " << gnss_position.latitude);
            ROS_INFO_STREAM("Longitude " << gnss_position.longitude);
            ROS_INFO_STREAM("Timestamp " << gnss_position.timestamp);
            ROS_INFO("----------------------------");
            new_gnss = false;
        }
        // pose.is_estimated = true;
        // pose.x = 1;
        // pose.y = 1;
        // pose.z = 1;
        // pose.bearing = 27;
        // pose.timestamp = 1234;

        // publisher.publish(pose);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}