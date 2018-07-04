// Copyright (c) 2016 The UUV Simulator Authors.
// All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <ros/ros.h>
#include <boost/bind.hpp>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geographic_msgs/GeoPoint.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <uuv_plume_msgs/ParticleConcentration.h>
#include <uuv_plume_msgs/Salinity.h>
#include <memory>
#include <string>
#include <chrono>
#include <random>

namespace uuv_plume_simulator
{

#define CONCENTRATION_UNIT_PPT "ppt"
#define CONCENTRATION_UNIT_PPM "ppm"
#define CONCENTRATION_UNIT_PSU "psu"

class CPCSensor
{
    /// \brief Class constructor
    public: CPCSensor();

    /// \brief Class destructor
    public: ~CPCSensor();

    /// \brief Update the output concentration and salinity topics
    protected: void OnSensorUpdate(const ros::TimerEvent&);

    /// \brief Update callback from the plume particles
    protected: void OnPlumeParticlesUpdate(
      const sensor_msgs::PointCloud::ConstPtr &_msg);

    /// \brief Update the odometry callback
    protected: void OnOdometryUpdate(
        const nav_msgs::Odometry::ConstPtr &_msg);

    /// \brief Update the GPS update callback
    protected: void OnGPSUpdate(
        const sensor_msgs::NavSatFix::ConstPtr &_msg);

    /// \brief Flag to ensure the cloud and measurement update don't coincide
    protected: bool updatingCloud;

    /// \brief Gamma velocity parameter for the smoothing function
    protected: double gamma;

    /// \brief Sensor gain
    protected: double gain;

    /// \brief Radius of the kernel to identify particles that will be
    /// taken into account in the concentration computation
    protected: double smoothingLength;

    /// \brief Salinity unit to be used. Options are
    ///  - PPT (parts per thousand)
    ///  - PPM (parts per million)
    ///  - PSU (practical salinity unit)
    protected: std::string salinityUnit;

    /// \brief Sensor saturation
    protected: double saturation;

    /// \brief Flag that will allow storing the geodetic coordinates with the
    /// measurement message
    protected: bool useGeoCoordinates;

    /// \brief Flag to activate publishing the simulated salinity
    protected: bool publishSalinity;

    /// \brief Default salinity value for the fluid e.g. sea water
    protected: double referenceSalinityValue;

    /// \brief Default salinity value for the plume
    protected: double plumeSalinityValue;

    /// \brief Set to true to avoid particle update
    protected: bool updateMeasurement;

    /// \brief Output topic's update rate
    protected: double updateRate;

    /// \brief Name of the sensor frame
    protected: std::string sensorFrameID;

    /// \brief Flag set to true after the first set of plume particles is
    /// received
    protected: bool areParticlesInit;

    /// \brief Flag set if the sensor position update must be read from the
    /// vehicle's odometry input topic
    protected: bool useOdom;

    /// \brief Flag set if the sensor position update must be read from the
    /// vehicle's GPS topic
    protected: bool useGPS;

    /// \brief Flag set if the TF update wrt the sensor frame ID
    protected: bool useTFUpdate;

    /// \brief Measured Cartesian position
    protected: geometry_msgs::Vector3 cartPos;

    /// \brief Measured geodetic position
    protected: geographic_msgs::GeoPoint geoPos;

    /// \brief Subscriber for the plume particle point cloud
    protected: ros::Subscriber particlesSub;

    /// \brief Subscriber for odometry topic
    protected: ros::Subscriber odometrySub;

    /// \brief Subscriber to the GPS update topic
    protected: ros::Subscriber gpsSub;

    /// \brief Output topic for particle concentration
    protected: ros::Publisher concentrationPub;

    /// \brief Output topic for salinity
    protected: ros::Publisher salinityPub;

    /// \brief ROS node handle
    protected: std::shared_ptr<ros::NodeHandle> nodeHandle;

    /// \brief TF buffer instance
    protected: tf2_ros::Buffer tfBuffer;

    /// \brief TF listener pointer
    protected: std::shared_ptr<tf2_ros::TransformListener> tfListener;

    /// \brief Local Cartesian projection
    protected: GeographicLib::LocalCartesian projection;

    /// \brief Plume concentration message
    protected: uuv_plume_msgs::ParticleConcentration concentrationMsg;

    /// \brief Salinity message
    protected: uuv_plume_msgs::Salinity salinityMsg;

    /// \brief Sensor update timer
    protected: ros::Timer updateTimer;

    /// \brief Pseudo random number generator
    protected: std::default_random_engine rndGen;

    /// \brief Normal distribution describing the noise model
    protected: std::normal_distribution<double> noiseModel;

    /// \brief Noise amplitude
    protected: double noiseAmp;

    /// \brief Noise standard deviation
    protected: double noiseSigma;
};
}
