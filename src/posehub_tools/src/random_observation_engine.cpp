#include <posehub_tools/random_observation_engine.hpp>
#include <ros/ros.h>
/* This library provides two ways to define a noisy camera model:

*/

SensorNoisyReading::SensorNoisyReading(const std::string &sensor_name,
                                       const std::vector<std::string> &topics,
                                       ros::NodeHandle &nh_,
                                       const double *noise_std_distr[36]) : sensor_name(sensor_name), topics(topics), nh(nh_)
/* noise_std_distr could be 36 in the lenght dimension*/
{
    // Initialize the sensor noisy reading with given noise and name
    dimension = 36;

    noise_covariance = std::vector<std::vector<double>>(6, std::vector<double>(6, 0.0));

    for (int i = 0; i < int(dimension); i++) //
    {
        noise_covariance[i / 6][i % 6] = *noise_std_distr[i];
    }

    for (int topic_num = 0; topic_num < topics.size(); topic_num++)
    {
        // Subscribe to the topics
        camera_subs.push_back(nh.subscribe(topics[topic_num], topics.size(), &SensorNoisyReading::SensorReadingCallback, this));
    }
}

SensorNoisyReading::SensorNoisyReading(const std::string &sensor_name,
                                       const std::vector<std::string> &topics,
                                       ros::NodeHandle &nh_) : sensor_name(sensor_name), topics(topics), nh(nh_)
{
    // Initialize the sensor noisy reading with default noise and given name
    this->sensor_name = sensor_name;
}

SensorNoisyReading::~SensorNoisyReading()
{
    sensor_name.clear();
    noise_covariance.clear();
}

void SensorNoisyReading::SensorReadingCallback(geometry_msgs::PoseConstPtr &msg)
{
    // Callback function to get the sensor reading
    setSensorReading(msg);

    // Apply noise to the sensor reading
    applyNoisetoReading();
}

void SensorNoisyReading::setSensorReading(const geometry_msgs::PoseConstPtr &pose)
{
    // Set the sensor reading, subsribe from the topic
    this->sensor_reading = *pose;
}

void SensorNoisyReading::setSensorNoise(const double *noise_std_distr, const int &dim)
{
    // Set the sensor noise
    // dim(NxN) is the size of the noise covariance matrix, 36 for 6D pose
    dimension = dim;

    noise_covariance = std::vector<std::vector<double>>(sqrt(dimension), std::vector<double>(sqrt(dimension), 0.0));

    for (int i = 0; i < int(dimension); i++) //
    {
        noise_covariance[i / int(sqrt(dimension))][i % int(sqrt(dimension))] = *noise_std_distr;
    }
}

void SensorNoisyReading::applyNoisetoReading()
{
    // Apply noise to the sensor reading
    noisy_reading.pose = sensor_reading;

    for (int i = 0; i < int(dimension); i++) //
    {
        noisy_reading.covariance[i] = noise_covariance[i / int(sqrt(dimension))][i % int(sqrt(dimension))];
    }

    // Publish the noisy reading
}
