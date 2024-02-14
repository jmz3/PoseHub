#include <ros/ros.h>
#include <random>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>

// define the sensor reading class
class SensorNoisyReading
{
public:
    SensorNoisyReading(const std::string &sensor_name, const double *noise_std_dev[36]);
    ~SensorNoisyReading();

    void setSensorReading(const geometry_msgs::Pose &pose);
    void setSensorNoise(const double &noise_std_dev);
    void applyNoisetoReading();

public:
    // Public members
    geometry_msgs::PoseWithCovariance noisy_reading;
    geometry_msgs::Pose sensor_reading;
    std::string sensor_name;

private:
};
