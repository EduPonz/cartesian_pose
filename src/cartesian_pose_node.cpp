#include <ros/console.h>
#include "cartesian_pose/CartesianPose.h"
#include "ros/ros.h"
// #include "gnss_l86_lib.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cartesian_pose_node");
    ros::NodeHandle n;
    ros::Publisher publisher = n.advertise<cartesian_pose::CartesianPose>("cartesian_pose", 1000);
    ros::Rate loop_rate(0.5);
    int i = 0;
    cartesian_pose::CartesianPose pose;

    while (ros::ok())
    {
        ROS_INFO_STREAM("I'm the cartesian_pose_node " << i);
        i++;
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