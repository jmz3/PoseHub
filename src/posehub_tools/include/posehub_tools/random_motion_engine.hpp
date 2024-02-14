#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Accel.h>
#include <string.h>
#include <vector>
#include <random>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>

class ObjectInformation
{
public:
    std::string object_name;
    geometry_msgs::Pose pose;
    geometry_msgs::Twist velocity;
    geometry_msgs::Accel acceleration;

    ObjectInformation();

    ~ObjectInformation();

    void setPose(double x, double y, double z, double qx, double qy, double qz, double qw);

    void setPose(tf2::Quaternion q);

    void setVelocity(double vx, double vy, double vz, double wx, double wy, double wz);

    void setAcceleration(double ax, double ay, double az, double awx, double awy, double awz);
};

class ObjectRandomMotionEngine
{
public:
    ObjectRandomMotionEngine(std::string const &name, const double &T = 0.01, const double &MAX_P = 1, const double &MAX_V = 1, const double &MAX_A = 1);
    ~ObjectRandomMotionEngine();

    void start();
    void update();
    void stop();

public:
    // Public members
    ObjectInformation object_state;
    double max_position;
    double max_velocity;
    double max_acceleration;
    double deltaT;

private:
    // Private members
    // ...
    const double ACC_SCALE = 20.0;
    std::random_device rd;
    std::mt19937 rng;
    std::uniform_real_distribution<> urd;
    tf2::Quaternion q;
};

class ObjectStaticEngine
{
public:
    ObjectStaticEngine(std::string const &name, geometry_msgs::PosePtr &pose, const double &T = 0.01);
    ObjectStaticEngine(std::string const &name, const double &T = 0.01);
    ~ObjectStaticEngine();

    void start(); // no need to implement this function since the object is static
    void update(geometry_msgs::PosePtr &pose);

public:
    // Public members
    ObjectInformation object_state;
    double deltaT;

private:
    // Private members
    // ...
    geometry_msgs::PoseConstPtr static_pose;
};