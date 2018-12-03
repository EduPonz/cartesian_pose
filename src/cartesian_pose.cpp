#include <assert.h>
#include <math.h>
#include <ros/console.h>
#include "cartesian_pose/cartesian_pose.h"

// ******************************** CONSTRUCTORS-DESTRUCTORS *******************************
CartesianPose::CartesianPose(gps_position magnetic_north, gps_position ref, coordinates_2d initial_vel, coordinates_2d initial_acc, float initial_bearing_mag)
{
    assert(check_gps_position_(magnetic_north));
    assert(check_gps_position_(ref));
    assert(check_angle_(initial_bearing_mag));

    magnetic_north_ = radians_(magnetic_north);
    ref_ = radians_(ref);
    last_gps_ = radians_(ref);

    set_velocity(initial_vel);
    set_acceleration_(initial_acc);
    declination_ = magnetic_declination_(ref_, magnetic_north_);
    set_last_bearing_(initial_bearing_mag);
    set_last_yaw_vel_(0);
    set_last_yaw_acc_(0);

    last_cartesian_.position = cartesian_position_(ref);
    last_cartesian_.bearing = last_bearing_;
    last_cartesian_.timestamp = ref.timestamp;
}

CartesianPose::~CartesianPose() { }

// **************************************** PRIVATE ****************************************
float CartesianPose::bearing_(gps_position origin, gps_position destination)
{
    float delta = destination.longitude - origin.longitude;
    float s = cos(destination.latitude) * sin(delta);
    float c = cos(origin.latitude) * sin(destination.latitude) - sin(origin.latitude) * cos(destination.latitude) * cos(delta);
    float bearing = atan2(s, c);
    if (bearing < 0)
        bearing += 360;
    return bearing;
}

float CartesianPose::magnetic_declination_(gps_position gps, gps_position magnetic_north)
{
    return bearing_(gps, magnetic_north);
}

coordinates_2d CartesianPose::cartesian_position_(gps_position gps)
{
    assert(check_gps_position_(gps));
    gps = radians_(gps);

    float delta_phi = gps.latitude - ref_.latitude;
    float delta_lamdda = gps.longitude - ref_.longitude;
    float first = pow(sin(delta_phi / 2), 2);
    float last = pow(sin(delta_lamdda / 2), 2);
    float central_angle = 2 * asin(sqrt(first + cos(ref_.latitude) * cos(gps.latitude) * last));
    float great_circle_dist = EARTH_R * central_angle;
    float polar_angle = bearing_(ref_, gps);

    coordinates_2d position;
    position.x = great_circle_dist * cos(polar_angle);
    position.y = great_circle_dist * sin(polar_angle);
    return position;
}

bool CartesianPose::check_angle_(float degrees)
{
    if (degrees < -360 || degrees > 360)
        return false;
    else
        return true;
}

bool CartesianPose::check_gps_position_(gps_position gps)
{
    if (gps.latitude < -360 || gps.latitude > 360)
        return false;
    else if (gps.longitude < -360 || gps.longitude > 360)
        return false;
    else if (gps.timestamp < 0)
        return false;
    else
        return true;    
}

float CartesianPose::degrees_(float radians)
{
    return (radians * 180) / M_PI;
}

gps_position CartesianPose::degrees_(gps_position gps)
{
    gps.latitude = degrees_(gps.latitude);
    gps.longitude = degrees_(gps.longitude);
    return gps;
}

float CartesianPose::radians_(float degrees)
{
    return (degrees * M_PI) / 180;
}

gps_position CartesianPose::radians_(gps_position gps)
{
    gps.latitude = radians_(gps.latitude);
    gps.longitude = radians_(gps.longitude);
    return gps;
}

void CartesianPose::set_acceleration_(coordinates_2d acceleration)
{
    last_acceleration_ = acceleration;
}

void CartesianPose::set_last_cartesian_(cart_pose pose)
{
    last_cartesian_ = pose;
}

void CartesianPose::set_last_bearing_(float bearing_mag)
{
    assert(check_angle_(bearing_mag));
    last_bearing_ = radians_(bearing_mag) + declination_;
}

void CartesianPose::set_last_yaw_vel_(float yaw_vel)
{
    last_yaw_vel_ = yaw_vel;
}

void CartesianPose::set_last_yaw_acc_(float yaw_acc)
{
    last_yaw_acc_ = yaw_acc;
}
// **************************************** PUBLIC *****************************************
cart_pose CartesianPose::cartesian_pose(gps_position gps, float bearing_mag)
{
    assert(check_angle_(bearing_mag));

    float bearing = radians_(bearing_mag) + declination_;

    cart_pose pose;
    pose.position = cartesian_position_(gps);
    pose.bearing = last_bearing_;
    pose.timestamp = gps.timestamp;

    // float delta_time = ((float)pose.timestamp - (float)last_cartesian_.timestamp) / 1000;
    unsigned long delta_time_t = (pose.timestamp - last_cartesian_.timestamp);
    float delta_time = (float)delta_time_t / 1000.0;
    ROS_INFO_STREAM("delta time " << delta_time);

    coordinates_2d temp_vel;
    ROS_INFO_STREAM("dif " << (float)(pose.position.x - last_cartesian_.position.x));
    temp_vel.x = (float)(pose.position.x - last_cartesian_.position.x) / delta_time;
    temp_vel.y = (float)(pose.position.y - last_cartesian_.position.y) / delta_time;
    ROS_INFO_STREAM("X vel " << temp_vel.x);

    coordinates_2d temp_acc;
    temp_acc.x = (float)(temp_vel.x - last_velocity_.x) / delta_time;
    temp_acc.y = (float)(temp_vel.y - last_velocity_.y) / delta_time;
    ROS_INFO_STREAM("X acc " << temp_acc.x);

    float delta_bearing = bearing - last_bearing_;
    float temp_yaw_vel = delta_bearing / delta_time;
    float delta_yaw_vel = temp_yaw_vel - last_yaw_vel_;
    float temp_yaw_acc = delta_yaw_vel / delta_time;

    set_last_cartesian_(pose);
    set_velocity(temp_vel);
    set_acceleration_(temp_acc);
    set_last_yaw_vel_(temp_yaw_vel);
    set_last_yaw_acc_(temp_yaw_acc);

    return pose;
}

cart_pose CartesianPose::cartesian_pose(imu_data imu)
{
    assert(imu.timestamp > 0);
    set_last_bearing_(imu.bearing);

    ROS_INFO_STREAM("imu time " << imu.timestamp);
    ROS_INFO_STREAM("last time " << last_cartesian_.timestamp);
    // double delta_time = (imu.timestamp - last_cartesian_.timestamp) / 1000;
    unsigned long delta_time_t = (imu.timestamp - last_cartesian_.timestamp);
    float delta_time = (float)delta_time_t / 1000.0;
    ROS_INFO_STREAM("delta time_t " << delta_time_t);
    ROS_INFO_STREAM("delta time " << delta_time);

    cart_pose pose;
    // pose.position.x = (float)last_cartesian_.position.x
    //                 + (float)last_velocity_.x * delta_time
    //                 + (float)imu.acceleration.x * pow(delta_time, 2);

    ROS_INFO_STREAM("X vel last " << last_velocity_.x);
    pose.position.x = last_cartesian_.position.x + last_velocity_.x;

    ROS_INFO_STREAM("X pose " << pose.position.x);

    pose.position.y = (float)last_cartesian_.position.y
                    + (float)last_velocity_.y * delta_time
                    + (float)imu.acceleration.y * pow(delta_time, 2);

    ROS_INFO_STREAM("Y pose " << pose.position.y);

    pose.bearing = last_bearing_;
    pose.timestamp = imu.timestamp;

    ROS_INFO_STREAM("Bearing " << pose.bearing);
    ROS_INFO_STREAM("timestamp " << pose.timestamp);

    coordinates_2d vel;
    vel.x = last_velocity_.x + imu.acceleration.x * delta_time;
    vel.y = last_velocity_.y + imu.acceleration.y * delta_time;
    ROS_INFO_STREAM("X vel " << last_velocity_.x);

    float yaw_vel = radians_(imu.yaw_vel);
    float yaw_acc = (yaw_vel - last_yaw_vel_) * delta_time; 

    set_last_cartesian_(pose);
    set_velocity(vel);
    set_acceleration_(imu.acceleration);
    set_last_yaw_vel_(yaw_vel);
    set_last_yaw_acc_(yaw_acc);

    return pose;
}

cart_pose CartesianPose::get_last_cartesian()
{
    return last_cartesian_;
}

float CartesianPose::get_magnectic_declination()
{
    return declination_;
}

gps_position CartesianPose::get_magnetic_north_gps()
{
    return degrees_(magnetic_north_);
}

float CartesianPose::get_last_bearing()
{
    return last_bearing_;
}

gps_position CartesianPose::get_last_gps()
{
    return degrees_(last_gps_);
}

gps_position CartesianPose::get_gps_ref()
{
    return degrees_(ref_);
}

coordinates_2d CartesianPose::get_acceleration()
{
    return last_acceleration_;
}

coordinates_2d CartesianPose::get_velocity()
{
    return last_velocity_;
}

float CartesianPose::get_yaw_acceleration()
{
    return last_yaw_acc_;
}

float CartesianPose::get_yaw_velocity()
{
    return last_yaw_vel_;
}

bool CartesianPose::set_magnetic_declination(float declination)
{
    if (!check_angle_(declination_))
        return false;
    else
    {
        declination_ = radians_(declination);
        return true;
    }
}

bool CartesianPose::set_gps_ref(gps_position gps)
{
    if (!check_gps_position_(gps))
        return false;
    else
    {
        ref_ = radians_(gps);
        return true;
    }
}

bool CartesianPose::set_magnetic_north_gps(gps_position gps)
{
    if (!check_gps_position_(gps))
        return false;
    else
    {
        magnetic_north_ = radians_(gps);
        return true;
    }
}

void CartesianPose::set_velocity(coordinates_2d velocity)
{
    last_velocity_.x = velocity.x;
    last_velocity_.y = velocity.y;
    ROS_INFO_STREAM("------X vel------------- " << velocity.x);
}
