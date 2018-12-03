#include <fstream>
#include <string>
#include <ros/console.h>
#include "ros/ros.h"
#include "cartesian_pose/CartesianLog.h"
#include "cartesian_pose/cartesian_pose.h"
#include "gnss_l86_interface/gnss_l86_lib.h"
#include "gnss_l86_interface/GnssData.h"
#include "imu_interface/gy_88_lib.h"
#include "imu_interface/Gy88Data.h"
#include "catamaran_controller/LogInstruction.h"

gps_position gps_data;
imu_data imu_data;
bool new_imu = false;
bool new_gps = false;
int instruction = 0;

void gnss_data_callback(const gnss_l86_interface::GnssData::ConstPtr& gnss_msg)
{
    gps_data.latitude = gnss_msg->latitude;
    gps_data.longitude = gnss_msg->longitude;
    gps_data.timestamp = gnss_msg->timestamp;
    new_gps = true;
}

void imu_data_callback(const imu_interface::Gy88Data::ConstPtr& imu_msg)
{
    imu_data.acceleration.x = imu_msg->si_accel_x;
    imu_data.acceleration.y = imu_msg->si_accel_y;
    imu_data.yaw_vel = imu_msg->gyro_z;
    imu_data.bearing = imu_msg->compass_angle;
    imu_data.timestamp = imu_msg->timestamp;
    new_imu = true;
}

void instruction_callback(const catamaran_controller::LogInstruction::ConstPtr& instruction_msg)
{
    instruction = instruction_msg->instruction;
}

int get_file_number(std::string file_name)
{
    std::string line;
    std::ifstream count_file(file_name);
    int count = (int)std::getline(count_file, line) + 1;
    count_file.close();
    return count;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cartesian_pose_node");
    ros::NodeHandle n;
    ros::Publisher publisher = n.advertise<cartesian_pose::CartesianLog>("cartesian_log", 1000);
    ros::Subscriber gnss_sub = n.subscribe("gnss_data", 1000, gnss_data_callback);
    ros::Subscriber imu_sub = n.subscribe("gy88_data", 1000, imu_data_callback);
    ros::Subscriber catamaran_sub = n.subscribe("log_instruction", 1000, instruction_callback);
    ros::Rate loop_rate(20);

    cartesian_pose::CartesianLog cartesian_log;

    bool is_first_gps = true;

    coordinates_2d vel;
    vel.x = 0.0;
    vel.y = 0.0;

    coordinates_2d acc;
    acc.x = 0.0;
    acc.y = 0.0;

    CartesianPose pose(gps_data, gps_data, vel, acc, 0);
    cart_pose cartesian_pose;

    bool new_data = false;
    bool file_close = true;
    std::ofstream file;
    std::string directory = "/home/ubuntu/catkin_ws/src/cartesian_pose/log/";
    std::string file_name;

    while (ros::ok())
    {
        if (file_close)
        {
            switch(instruction)
            {
                case 2:
                    surge_file = directory + "surge_counter.txt";
                    int count = get_file_number(surge_file);
                    file_name = directory + "surge_damping_test_" + std::to_strin(count) + ".csv";
                    file.open(file_name);
                    file_close = false;
                    break;
                case 4:
                    file_name = directory + "yaw_damping_test.csv";
                    file.open(file_name);
                    file_close = false;
                    break;
            }
        }
        else if (instruction == 0)
        {
            file.close();
            file_close = true;
        }

        if (is_first_gps && new_imu && new_gps)
        {
            pose = CartesianPose(gps_data, gps_data, vel, acc, imu_data.bearing);
            cartesian_pose = pose.get_last_cartesian();
            is_first_gps = false;
            new_gps = false;
            new_imu = false;
            new_data = true;
        }
        else if (new_gps && !is_first_gps)
        {
            cartesian_pose = pose.cartesian_pose(gps_data, imu_data.bearing);
            cartesian_log.ready_to_log = true;
            publisher.publish(cartesian_log);
            new_gps = false;
            new_data = true;
        }
        else if (new_imu && !is_first_gps)
        {
            cartesian_pose = pose.cartesian_pose(imu_data);
            new_imu = false;
            new_data = true;
        }

        if (new_data && !file_close)
        {
            file << pose.get_speed() << ";"
                 << pose.get_yaw_velocity() << ";"
                 << cartesian_pose.timestamp << std::endl;
            new_data = false;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
