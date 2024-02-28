/* This is a ROS node that creates a fake sensor noise for the input poses.
It subscribes to all the topics that match the specified pattern, which is "object" by default,
 and adds noise to the input poses to mimic the real sensor readings. */

#include <ros/ros.h>
#include <ros/master.h>
#include <vector>
#include <string>
#include <regex>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <posehub_tools/random_observation_engine.hpp>
// Function to get the list of topics that match the input pattern using regular expressions
std::vector<std::string> getTopicsWithPattern(const std::string &pattern)
{
    std::vector<ros::master::TopicInfo> all_topics;
    std::vector<std::string> filtered_topics;

    // Get the list of all topics
    ros::master::getTopics(all_topics);

    // Compile the regular expression
    std::regex regex_pattern(".*" + pattern + ".*");

    // Filter topics that match the input pattern
    for (const auto &topic_info : all_topics)
    {
        try
        {
            if (std::regex_match(topic_info.name, regex_pattern))
            {
                // subtract the namespace from the topic name
                std::string::size_type pos = topic_info.name.find_last_of('/');
                if (pos != std::string::npos)
                {
                    filtered_topics.push_back(topic_info.name.substr(pos + 1));
                }
                else
                {
                    filtered_topics.push_back(topic_info.name);
                }
            }
        }
        catch (std::regex_error &e)
        {
            ROS_ERROR("Regex error: %s", e.what());
            // Handle regex error (e.g., log, throw exception, etc.)
        }
    }

    return filtered_topics;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fake_sensor_noise_node");
    ros::NodeHandle nh;

    std::vector<std::string> filtered_topics = getTopicsWithPattern("object");

    // print the list of filtered topics
    for (const auto &topic : filtered_topics)
    {
        ROS_INFO("Filtered topic: %s", topic.c_str());
    }

    // Get the sensor name from the parameter server, default is "HoloLens"
    std::string sensor_name;
    nh.param<std::string>("fake_sensor_noise_node/sensor_name", sensor_name, "HoloLens");

    ROS_INFO("The sensor name is: %s", sensor_name.c_str());

    SensorNoisyReading noisy_camera_(sensor_name, filtered_topics, nh);
    noisy_camera_.setSensorNoise(new double[36]{0.1, 0, 0, 0, 0, 0,
                                                0, 0.1, 0, 0, 0, 0,
                                                0, 0, 0.1, 0, 0, 0,
                                                0, 0, 0, 0.1, 0, 0,
                                                0, 0, 0, 0, 0.1, 0,
                                                0, 0, 0, 0, 0, 0.1},
                                 36);

    ros::Rate rate(100);

    // Get the list of topics that match the pattern

    // Print the list of filtered topics
    while (nh.ok())
    {
        noisy_camera_.updateSensorReading();
        rate.sleep();
    }

    return 0;
}
