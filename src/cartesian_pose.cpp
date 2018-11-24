#include <assert.h>
#include <math.h>
#include "cartesian_pose/cartesian_pose.h"

// ******************************** CONSTRUCTORS-DESTRUCTORS *******************************
CartesianPose::CartesianPose(gps_position magnetic_north, gps_position ref, coordinates_2d initial_velocity)
{
    assert(check_gps_position_(magnetic_north));
    assert(check_gps_position_(ref));

    magnetic_north_ = radians_(magnetic_north);
    ref_ = radians_(ref);
    last_gps_ = radians_(ref);
    last_velocity_.x = initial_velocity.x;
    last_velocity_.y = initial_velocity.y;
    declination_ = calculate_magnetic_declination_(ref_, magnetic_north_);
}

CartesianPose::~CartesianPose() { }

// **************************************** PRIVATE ****************************************
float CartesianPose::calculate_bearing_(gps_position origin, gps_position destination)
{
    float delta = destination.longitude - origin.longitude;
    float s = cos(destination.latitude) * sin(delta);
    float c = cos(origin.latitude) * sin(destination.latitude) - sin(origin.latitude) * cos(destination.latitude) * cos(delta);
    return atan2(s, c);
}

float CartesianPose::calculate_magnetic_declination_(gps_position gps, gps_position magnetic_north)
{
    return calculate_bearing_(gps, magnetic_north);
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

// **************************************** PUBLIC *****************************************
cart_pose CartesianPose::calculate_cartesian(gps_position gps, float bearing)
{
    assert(check_gps_position_(gps));
    assert(check_angle_(bearing));
    gps = radians_(gps);

    float delta_phi = gps.latitude - ref_.latitude;
    float delta_lamdda = gps.longitude - ref_.longitude;
    float first = pow(sin(delta_phi / 2), 2);
    float last = pow(sin(delta_lamdda / 2), 2);
    float central_angle = 2 * asin(sqrt(first + cos(ref_.latitude) * cos(gps.latitude) * last));
    float great_circle_dist = EARTH_R * central_angle;
    float polar_angle = calculate_bearing_(ref_, gps);
    
    cart_pose pose;
    pose.position.x = great_circle_dist * sin(polar_angle);
    pose.position.y = great_circle_dist * cos(polar_angle);
    pose.velocity = last_velocity_;
    pose.acceleration = last_acceleration_;
    pose.bearing = bearing;
    pose.timestamp = gps.timestamp;
    last_cartesian_ = pose;
    return pose;
}

cart_pose CartesianPose::get_last_cartesian()
{
    return last_cartesian_;
}

float CartesianPose::get_magnectic_declination()
{
    return degrees_(declination_);
}

gps_position CartesianPose::get_north_magnetic_gps()
{
    return degrees_(magnetic_north_);
}

gps_position CartesianPose::get_last_gps()
{
    return degrees_(last_gps_);
}

gps_position CartesianPose::get_gps_ref()
{
    return degrees_(ref_);
}

coordinates_2d CartesianPose::get_velocity()
{
    return last_velocity_;
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

bool CartesianPose::set_north_magnetic_gps(gps_position gps)
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
    last_velocity_ = velocity;
}
