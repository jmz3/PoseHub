#include <ros/ros.h>
#include <random>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>

// define the sensor reading class
class SensorNoisyReading
{
public:
    SensorNoisyReading(const std::string &sensor_name,
                       const std::vector<std::string> &topics,
                       ros::NodeHandle &nh_,
                       const double *noise_std_distr[36]);
    // Constructor to initialize the sensor noisy reading
    /* noise_std_distr is recommended to have 36 elements for 6D pose
       But it could be 3x3 or 7x7 for position and (position, orientation) respectively */
    SensorNoisyReading(const std::string &sensor_name,
                       const std::vector<std::string> &topics,
                       ros::NodeHandle &nh_);
    ~SensorNoisyReading();

    void setSensorReading(const geometry_msgs::PoseConstPtr &pose);
    void setSensorNoise(const double *noise_std_distr, const int &dim); // Set the sensor noise covariance
    /* Pass the noise convariance by giving the pointer */
    void applyNoisetoReading();
    void SensorReadingCallback(geometry_msgs::PoseConstPtr &msg);

public:
    // Public members
    geometry_msgs::PoseWithCovariance noisy_reading;
    geometry_msgs::Pose sensor_reading;
    std::string sensor_name;
    std::vector<std::string> topics;

private:
    // Private members
    std::vector<std::vector<double>> noise_covariance;
    size_t dimension; // Size of the noise covariance matrix, 36 for 6D pose
    ros::NodeHandle nh;
    std::vector<ros::Subscriber> camera_subs;
    std::vector<ros::Publisher> camera_pubs;
};
