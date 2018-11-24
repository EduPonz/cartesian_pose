#define EARTH_R 6371000
#define PI 3.14159265

struct gps_position
{
    float latitude;
    float longitude;
    float timestamp;
};

struct coordinates_2d
{
    float x;
    float y;
};

struct cart_pose
{
    coordinates_2d position;
    coordinates_2d velocity;
    coordinates_2d acceleration;
    float bearing;
    float timestamp;
};

class CartesianPose
{
    private:
        gps_position magnetic_north_;
        gps_position ref_;
        gps_position last_gps_;
        cart_pose last_cartesian_;
        coordinates_2d last_velocity_;
        coordinates_2d last_acceleration_;
        float declination_;
        float calculate_bearing_(gps_position origin, gps_position destination);
        float calculate_magnetic_declination_(gps_position gps, gps_position magnetic_north);
        bool check_angle_(float degrees);
        bool check_gps_position_(gps_position gps);
        float degrees_(float radians);
        gps_position degrees_(gps_position gps);
        float radians_(float degrees);
        gps_position radians_(gps_position gps);
    public:
        CartesianPose(gps_position magnetic_north, gps_position ref, coordinates_2d initial_velocity);
        ~CartesianPose();
        cart_pose calculate_cartesian(gps_position gps, float bearing);
        cart_pose estimate_cartesian(cart_pose cartesian_pose);
        coordinates_2d get_velocity();
        gps_position get_north_magnetic_gps();
        gps_position get_last_gps();
        gps_position get_gps_ref();
        cart_pose get_last_cartesian();
        float get_magnectic_declination();
        bool set_magnetic_declination(float declination);
        bool set_gps_ref(gps_position gps);
        bool set_north_magnetic_gps(gps_position gps);
        void set_velocity(coordinates_2d velocity);
};
