// MIT License
//
// Copyright(c)[2024][Jiaming Zhang]
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files(the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <posehub_tools/random_observation_engine.hpp>
#include <ros/ros.h>
#include <iostream>
/* This library provides two ways to define a noisy camera model:
first, by providing the conventional noise covariance matrix (6x6) and the sensor frame name,
second, by providing the sensor name and the covariance matrix (any shape) is allowed to be set later.
*/

SensorNoisyReading::SensorNoisyReading(const std::string &sensor_frame_name,
                                       const std::vector<std::string> &child_frames,
                                       ros::NodeHandle &nh_,
                                       const double *noise_std_distr[36]) : sensor_frame_name(sensor_frame_name), child_frames(child_frames), nh(nh_)
/* noise_std_distr could be 36 in the lenght dimension*/
{
    // Initialize the sensor noisy reading with given noise and name
    dimension = 36;

    noise_covariance = std::vector<std::vector<double>>(6, std::vector<double>(6, 0.0));
    sensor_state.header.frame_id = sensor_frame_name;

    // the diagonal elements of the noise covariance matrix should be 1 by default
    noise_covariance[0][0] = 1.0;
    noise_covariance[1][1] = 1.0;
    noise_covariance[2][2] = 1.0;
    noise_covariance[3][3] = 1.0;
    noise_covariance[4][4] = 1.0;
    noise_covariance[5][5] = 1.0;

    for (int i = 0; i < int(dimension); i++) //
    {
        noise_covariance[i / 6][i % 6] = *noise_std_distr[i];
    }

    sensor_state_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
        sensor_frame_name + "_noisy",
        10);

    for (int frame_num = 0; frame_num < child_frames.size(); frame_num++)
    {
        // Create a publisher for each child frame
        object_state_pubs.push_back(nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
            sensor_frame_name +
                "/" +
                child_frames[frame_num] +
                "_noisy",
            10));
    }
}
SensorNoisyReading::SensorNoisyReading(const std::string &sensor_frame_name,
                                       const std::vector<std::string> &child_frames,
                                       ros::NodeHandle &nh_) : sensor_frame_name(sensor_frame_name), child_frames(child_frames), nh(nh_)
{
    sensor_state.header.frame_id = sensor_frame_name;

    sensor_state_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
        sensor_frame_name + "_noisy",
        10);

    for (int frame_num = 0; frame_num < child_frames.size(); frame_num++)
    {
        // Create a publisher for each child frame
        object_state_pubs.push_back(nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
            sensor_frame_name +
                "/" +
                child_frames[frame_num] +
                "_noisy",
            10));
    }
}

SensorNoisyReading::~SensorNoisyReading()
{
    sensor_frame_name.clear();
    noise_covariance.clear();
}

void SensorNoisyReading::setSensorNoise(const double *noise_std_distr, const int &dim)
{
    // Set the sensor noise
    // dim(NxN) is the size of the noise covariance matrix, 36 for 6D pose
    dimension = dim;

    noise_covariance = std::vector<std::vector<double>>(sqrt(dimension), std::vector<double>(sqrt(dimension), 0.0));

    for (int i = 0; i < int(dimension); i++) //
    {
        noise_covariance[i / int(sqrt(dimension))][i % int(sqrt(dimension))] = *(noise_std_distr + i);
    }

    // sanity check
    // print the noise covariance matrix
    ROS_INFO("The noise covariance matrix is set as follows:");
    for (int i = 0; i < int(sqrt(dimension)); i++)
    {
        for (int j = 0; j < int(sqrt(dimension)); j++)
        {
            std::cout << noise_covariance[i][j] << " ";

            if (j == int(sqrt(dimension)) - 1)
            {
                std::cout << std::endl;
            }
        }
    }
}

void SensorNoisyReading::updateSensorReading()
{

    try
    {
        sensor_tf_stamped_transform = tfBuffer.lookupTransform(
            "world",
            sensor_frame_name,
            ros::Time(0),
            ros::Duration(3.0));
        updateSensorState();
    }
    catch (tf2::TransformException &ex)
    {
        ROS_INFO("Waiting for the sensor to be staged in TF tree");
        ROS_ERROR("%s", ex.what());
    }
    // ROS_INFO("The sensor pose is updated");
    // Query the tf from the sensor frame to the target object frames, and apply noise to the sensor reading
    for (size_t frame_num = 0; frame_num < child_frames.size(); frame_num++)
    {
        try
        {
            object_tf_stamped_transform = tfBuffer.lookupTransform(
                sensor_frame_name,
                child_frames[frame_num],
                ros::Time(0),
                ros::Duration(3.0));

            updateObjectState(frame_num);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_INFO("Waiting for the object %s to be staged in TF tree", child_frames[frame_num].c_str());
            ROS_ERROR("%s", ex.what());
        }
    }
    // ROS_INFO("The object poses are updated");
}

void SensorNoisyReading::updateSensorState()
{
    // Set the sensor state, including the pose and its covariance
    sensor_state.pose.pose.position.x = sensor_tf_stamped_transform.transform.translation.x;
    sensor_state.pose.pose.position.y = sensor_tf_stamped_transform.transform.translation.y;
    sensor_state.pose.pose.position.z = sensor_tf_stamped_transform.transform.translation.z;
    sensor_state.pose.pose.orientation = sensor_tf_stamped_transform.transform.rotation;

    if (dimension == 36)
    {
        for (int i = 0; i < int(dimension); i++) //
        {
            sensor_state.pose.covariance[i] = noise_covariance[i / int(sqrt(dimension))][i % int(sqrt(dimension))];
        }
    }
    else
    {
        // covariance with other size to be implemented in the future
        ROS_INFO("The noise covariance matrix cannot be set for the sensor due to the incompatible dimension.");
    };

    sensor_state.header.stamp = ros::Time::now();
    sensor_state_pub.publish(sensor_state);
}

void SensorNoisyReading::updateObjectState(size_t frame_numbering_id)
{
    /* frame_numbering_id is for correct correspondence between the object transform and the publisher
     */
    object_state.pose.pose.position.x = object_tf_stamped_transform.transform.translation.x;
    object_state.pose.pose.position.y = object_tf_stamped_transform.transform.translation.y;
    object_state.pose.pose.position.z = object_tf_stamped_transform.transform.translation.z;
    object_state.pose.pose.orientation = object_tf_stamped_transform.transform.rotation;

    // Set the object state, including the pose and its covariance

    if (dimension == 36)
    {
        for (int i = 0; i < int(dimension); i++) //
        {
            object_state.pose.covariance[i] = noise_covariance[i / int(sqrt(dimension))][i % int(sqrt(dimension))];
        }
    }
    else
    {
        // covariance with other size to be implemented in the future
        ROS_INFO("The noise covariance matrix cannot be set for the sensor due to the incompatible dimension.");
    };

    object_state.header.stamp = ros::Time::now();
    object_state.header.frame_id = sensor_frame_name;

    object_state_pubs[frame_numbering_id].publish(object_state);
}
