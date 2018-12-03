#include <ros/console.h>
#include "ros/ros.h"
#include "gnss_l86_interface/GnssData.h"
#include "cartesian_pose/CartesianLog.h"
#include "cartesian_pose/cartesian_pose.h"
#include "gnss_l86_interface/gnss_l86_lib.h"

position gnss_position;
bool new_gnss = false;
int fix_state = 0;
int prev_fix_state = -1;

void gnss_data_callback(const gnss_l86_interface::GnssData::ConstPtr& gnss_msg)
{
    // gnss_position.latitude = gnss_msg->latitude;
    // gnss_position.longitude = gnss_msg->longitude;
    // gnss_position.fix = gnss_msg->fix;
    // gnss_position.number_of_satelites = gnss_msg->number_of_satelites;
    // gnss_position.horizontal_precision = gnss_msg->horizontal_precision;
    // gnss_position.altitude = gnss_msg->altitude;
    // gnss_position.timestamp = gnss_msg->timestamp;
    // new_gnss = true;
    fix_state = gnss_msg->fix;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cartesian_pose_node");
    ros::NodeHandle n;
    ros::Publisher publisher = n.advertise<cartesian_pose::CartesianLog>("cartesian_log", 1000);
    ros::Subscriber subscriber = n.subscribe("gnss_data", 1000, gnss_data_callback);
    ros::Rate loop_rate(1000);
    cartesian_pose::CartesianLog cartesian_log;

    // gps_position north;
    // north.latitude = 84.7;
    // north.longitude = 169.77;
    // north.timestamp = 0;

    // gps_position ref;
    // ref.latitude = 57.052968;
    // ref.longitude = 9.915675;
    // ref.timestamp = 0;

    // coordinates_2d vel;
    // vel.x = 0;
    // vel.y = 0;

    // coordinates_2d acc;
    // acc.x = 0;
    // acc.y = 0;

    // float bearing = -2.96452;
    // CartesianPose pose(north, ref, vel, acc, bearing);
    // cart_pose dest_c = pose.get_last_cartesian();

    // imu_data imu;
    // imu.acceleration.x = 3;
    // imu.acceleration.y = 0;
    // imu.yaw_vel = 180;
    // imu.bearing = 177;
    // imu.timestamp = 1000;

    // dest_c = pose.cartesian_pose(imu);

    // gps_position dest;
    // dest.latitude = 57.052968;
    // dest.longitude = 9.905196;
    // dest.timestamp = 301000;

    // dest_c = pose.cartesian_pose(dest, 87);

    while (ros::ok())
    {
        if (fix_state != prev_fix_state)
        {
            cartesian_log.ready_to_log = fix_state;
            publisher.publish(cartesian_log);
            prev_fix_state = fix_state;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
