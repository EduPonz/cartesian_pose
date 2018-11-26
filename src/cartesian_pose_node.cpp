#include <ros/console.h>
#include "ros/ros.h"
#include "gnss_l86_interface/GnssData.h"
#include "cartesian_pose/CartesianPose.h"
#include "cartesian_pose/cartesian_pose.h"
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

    ROS_INFO(" ");
    ROS_INFO("----------------------------------------------------------------");
    ROS_INFO("---------------------- Initital Conditions ---------------------");
    ROS_INFO("----------------------------------------------------------------");

    gps_position north;
    north.latitude = 84.7;
    north.longitude = 169.77;
    north.timestamp = 0;

    gps_position ref;
    ref.latitude = 57.052968;
    ref.longitude = 9.915675;
    ref.timestamp = 0;

    coordinates_2d vel;
    vel.x = 0;
    vel.y = 0;

    coordinates_2d acc;
    acc.x = 0;
    acc.y = 0;

    float bearing = -2.96452;
    CartesianPose pose(north, ref, vel, acc, bearing);
    cart_pose dest_c = pose.get_last_cartesian();

    ROS_INFO_STREAM("Declination -----> " << pose.get_magnectic_declination() << " radians");
    ROS_INFO_STREAM("Position X ------> " << dest_c.position.x           << " m");
    ROS_INFO_STREAM("Position Y ------> " << dest_c.position.y           << " m");
    ROS_INFO_STREAM("Bearing ---------> " << dest_c.bearing              << " rad");
    ROS_INFO_STREAM("Timestamp -------> " << dest_c.timestamp            << " ms");
    ROS_INFO_STREAM("Velocity X ------> " << pose.get_velocity().x       << " m/s");
    ROS_INFO_STREAM("Velocity Y ------> " << pose.get_velocity().y       << " m/s");
    ROS_INFO_STREAM("Acceleration X --> " << pose.get_acceleration().x   << " m/s^2");
    ROS_INFO_STREAM("Acceleration Y --> " << pose.get_acceleration().y   << " m/s^2");
    ROS_INFO_STREAM("Yaw velocity ----> " << pose.get_yaw_velocity()     << " rad/s");
    ROS_INFO_STREAM("Yaw accelration -> " << pose.get_yaw_acceleration() << " rad/s^2");

    ROS_INFO(" ");
    ROS_INFO("----------------------------------------------------------------");
    ROS_INFO("---------------------- Conditions From IMU ---------------------");
    ROS_INFO("----------------------------------------------------------------");

    imu_data imu;
    imu.acceleration.x = 3;
    imu.acceleration.y = 0;
    imu.yaw_vel = 180;
    imu.bearing = 177;
    imu.timestamp = 1000;

    dest_c = pose.cartesian_pose(imu);
    ROS_INFO_STREAM("Position X ------> " << dest_c.position.x           << " m");
    ROS_INFO_STREAM("Position Y ------> " << dest_c.position.y           << " m");
    ROS_INFO_STREAM("Bearing ---------> " << dest_c.bearing              << " rad");
    ROS_INFO_STREAM("Timestamp -------> " << dest_c.timestamp            << " ms");
    ROS_INFO_STREAM("Velocity X ------> " << pose.get_velocity().x       << " m/s");
    ROS_INFO_STREAM("Velocity Y ------> " << pose.get_velocity().y       << " m/s");
    ROS_INFO_STREAM("Acceleration X --> " << pose.get_acceleration().x   << " m/s^2");
    ROS_INFO_STREAM("Acceleration Y --> " << pose.get_acceleration().y   << " m/s^2");
    ROS_INFO_STREAM("Yaw velocity ----> " << pose.get_yaw_velocity()     << " rad/s");
    ROS_INFO_STREAM("Yaw accelration -> " << pose.get_yaw_acceleration() << " rad/s^2");

    ROS_INFO(" ");
    ROS_INFO("----------------------------------------------------------------");
    ROS_INFO("---------------------- Conditions From GPS ---------------------");
    ROS_INFO("----------------------------------------------------------------");

    gps_position dest;
    dest.latitude = 57.052968;
    dest.longitude = 9.905196;
    dest.timestamp = 301000;

    dest_c = pose.cartesian_pose(dest, 87);
    ROS_INFO_STREAM("Position X ------> " << dest_c.position.x           << " m");
    ROS_INFO_STREAM("Position Y ------> " << dest_c.position.y           << " m");
    ROS_INFO_STREAM("Bearing ---------> " << dest_c.bearing              << " rad");
    ROS_INFO_STREAM("Timestamp -------> " << dest_c.timestamp            << " ms");
    ROS_INFO_STREAM("Velocity X ------> " << pose.get_velocity().x       << " m/s");
    ROS_INFO_STREAM("Velocity Y ------> " << pose.get_velocity().y       << " m/s");
    ROS_INFO_STREAM("Acceleration X --> " << pose.get_acceleration().x   << " m/s^2");
    ROS_INFO_STREAM("Acceleration Y --> " << pose.get_acceleration().y   << " m/s^2");
    ROS_INFO_STREAM("Yaw velocity ----> " << pose.get_yaw_velocity()     << " rad/s");
    ROS_INFO_STREAM("Yaw accelration -> " << pose.get_yaw_acceleration() << " rad/s^2");

    ROS_INFO("----------------------------------------------------------------");
    return 0;

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
