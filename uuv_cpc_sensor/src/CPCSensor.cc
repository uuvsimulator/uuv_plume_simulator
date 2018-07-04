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

#include <uuv_cpc_sensor/CPCSensor.hh>

using namespace uuv_plume_simulator;

/////////////////////////////////////////////////
CPCSensor::CPCSensor()
{
    ROS_INFO("Particle concentration sensor is starting");

    this->nodeHandle.reset(new ros::NodeHandle("~"));

    // Create ROS node
    if (!ros::isInitialized())
    {
        ROS_ERROR("Not loading sensor plugin since ROS has not been properly initialized");
        throw 1;
    }

    // Set seed for the noise generator
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    this->rndGen = std::default_random_engine(seed);

    this->nodeHandle->getParam("~gamma", this->gamma);
    ROS_ASSERT(this->gamma > 0);
    ROS_INFO_NAMED("CPCSensor", "Gamma: %3f", this->gamma);

    this->nodeHandle->getParam("~gain", this->gain);
    ROS_ASSERT(this->gain >= 0);
    ROS_INFO_NAMED("CPCSensor", "Gain: %3f", this->gain);

    this->nodeHandle->getParam("~radius", this->smoothingLength);
    ROS_ASSERT(this->smoothingLength >= 0);
    ROS_INFO_NAMED("CPCSensor", "Radius [m]: %3f", this->smoothingLength);

    this->nodeHandle->getParam("~update_rate", this->updateRate);
    ROS_ASSERT(this->updateRate > 0);
    ROS_INFO_NAMED("CPCSensor", "Update rate [Hz]: %3f", this->updateRate);

    this->saturation = 1.0;
    if (this->nodeHandle->hasParam("~saturation"))
        this->nodeHandle->getParam("~saturation", this->saturation);
    ROS_ASSERT(this->saturation >= 0);

    this->noiseAmp = 0.0;
    if (this->nodeHandle->hasParam("~noise_amplitude"))
        this->nodeHandle->getParam("~noise_amplitude", this->noiseAmp);
    ROS_ASSERT(this->noiseAmp >= 0);

    this->noiseSigma = 0.0;
    if (this->nodeHandle->hasParam("~noise_sigma"))
        this->nodeHandle->getParam("~noise_sigma", this->noiseSigma);
    ROS_ASSERT(this->noiseSigma >= 0);

    // Initialize noise model
    this->noiseModel = std::normal_distribution<double>(0.0, this->noiseSigma);

    // Read the geodetic reference as WGS84
    this->useGeoCoordinates = false;
    if (this->nodeHandle->hasParam("~use_geo_coordinates"))
        this->nodeHandle->getParam(
            "~use_geo_coordinates", this->useGeoCoordinates);

    // Create the coordinate converter
    if (this->useGeoCoordinates)
    {
        GeographicLib::Geocentric earth(
            GeographicLib::Constants::WGS84_a(),
            GeographicLib::Constants::WGS84_f());

        double latitude = -1;
        double longitude = -1;
        ROS_ASSERT(this->nodeHandle->hasParam("~latitude"));
        ROS_ASSERT(this->nodeHandle->hasParam("~longitude"));

        this->nodeHandle->getParam("~latitude", latitude);
        this->nodeHandle->getParam("~longitude", longitude);

        this->projection = GeographicLib::LocalCartesian(
            latitude, longitude, 0, earth);
    }

    // Loading the salinity unit for the salinity output
    this->salinityUnit = CONCENTRATION_UNIT_PPT;
    if (this->nodeHandle->hasParam("~salinity_unit"))
        this->nodeHandle->getParam("~salinity_unit", this->salinityUnit);

    ROS_ASSERT(!this->salinityUnit.compare(CONCENTRATION_UNIT_PPT) ||
        !this->salinityUnit.compare(CONCENTRATION_UNIT_PPM) ||
        !this->salinityUnit.compare(CONCENTRATION_UNIT_PSU));

    // Loading the reference salinity value to be computed from the particle
    // concentration value
    if (this->nodeHandle->hasParam("~reference_salinity_value"))
        this->nodeHandle->getParam("~reference_salinity_value",
            this->referenceSalinityValue);
    else
    {
        // If no reference is given, the sea water reference is provided
        if (this->salinityUnit.compare(CONCENTRATION_UNIT_PPT) == 0)
          this->referenceSalinityValue = 35.0;
        else if (this->salinityUnit.compare(CONCENTRATION_UNIT_PPM) == 0)
          this->referenceSalinityValue = 35000.0;
        else
          this->referenceSalinityValue = 35.0;
    }

    this->updatingCloud = false;

    // Subscribing to the particles topic (remap the topic name as desired)
    this->particlesSub = this->nodeHandle->subscribe<sensor_msgs::PointCloud>(
        "particles", 1, &CPCSensor::OnPlumeParticlesUpdate, this);

    // Publish the particle concentration value
    this->concentrationPub = this->nodeHandle->advertise<
        uuv_plume_msgs::ParticleConcentration>(
        "concentration", 1);

    this->useOdom = false;
    this->useGPS = false;
    this->useTFUpdate = false;
    if (this->nodeHandle->hasParam("~use_odom"))
        this->nodeHandle->getParam("~use_odom", this->useOdom);

    if (this->nodeHandle->hasParam("~use_gps"))
        this->nodeHandle->getParam("~use_gps", this->useGPS);

    if (this->useOdom)
    {
        // Subscribe to the odometry topic
        this->odometrySub = this->nodeHandle->subscribe<nav_msgs::Odometry>(
            "odom", 1, &CPCSensor::OnOdometryUpdate, this);
        ROS_INFO("Using the odometry <nav_msgs::Odometry> as input for sensor position");
    }
    else if (this->useGPS && this->useGeoCoordinates)
    {
        // Subscribe to the GPS topic
        this->gpsSub = this->nodeHandle->subscribe<sensor_msgs::NavSatFix>(
            "gps", 1, &CPCSensor::OnGPSUpdate, this);
        ROS_INFO("Using the GPS <sensor_msgs::NatSatFix> as input for sensor position");
    }
    else
    {
        this->useTFUpdate = true;
        this->nodeHandle->getParam("~sensor_frame_id", this->sensorFrameID);
        ROS_INFO("Using the a frame ID as input for sensor position");
    }

    this->areParticlesInit = false;
    // Initializing the particle concentration message
    this->concentrationMsg.header.frame_id = "world";
    this->concentrationMsg.header.stamp = ros::Time::now();

    this->tfListener.reset(new tf2_ros::TransformListener(this->tfBuffer));

    // Initialize the salinity topic
    this->publishSalinity = false;
    if (this->nodeHandle->hasParam("~publish_salinity"))
        this->nodeHandle->getParam("~publish_salinity", this->publishSalinity);

    if (this->publishSalinity)
        this->salinityPub = this->nodeHandle->advertise<
            uuv_plume_msgs::Salinity>("salinity", 1);

    this->updateTimer = this->nodeHandle->createTimer(
        ros::Duration(1.0 / this->updateRate),
        boost::bind(&CPCSensor::OnSensorUpdate, this, _1));

    ROS_INFO("Particle concentration sensor running");
}

/////////////////////////////////////////////////
CPCSensor::~CPCSensor()
{

}

/////////////////////////////////////////////////
void CPCSensor::OnPlumeParticlesUpdate(
  const sensor_msgs::PointCloud::ConstPtr &_msg)
{
    if (this->concentrationPub.getNumSubscribers() > 0)
    {
        this->updateMeasurement = true;
        this->areParticlesInit = true;

        if (this->useTFUpdate)
        {
            // Read the current position of the sensor frame
            geometry_msgs::TransformStamped childTransform;
            std::string targetFrame = _msg->header.frame_id;
            std::string sourceFrame = this->sensorFrameID;
            try
            {
              childTransform = this->tfBuffer.lookupTransform(
                targetFrame, sourceFrame, ros::Time(0));
            }
            catch(tf2::TransformException &ex)
            {
              ROS_ERROR_NAMED("CPCSensor", "Transform between %s and %s",
                targetFrame.c_str(), sourceFrame.c_str());
              return;
            }

            this->cartPos.x = childTransform.transform.translation.x;
            this->cartPos.y = childTransform.transform.translation.y;
            this->cartPos.z = childTransform.transform.translation.z;
        }

        this->concentrationMsg.header.frame_id = _msg->header.frame_id;
        this->concentrationMsg.header.stamp = ros::Time::now();

        this->concentrationMsg.position.x = this->cartPos.x;
        this->concentrationMsg.position.y = this->cartPos.y;
        this->concentrationMsg.position.z = this->cartPos.z;

        if (this->useGeoCoordinates)
        {
            if (this->useGPS)
                this->concentrationMsg.geo_point = this->geoPos;
            else
            {
                double latitude, longitude, altitude;
                this->projection.Reverse(
                    this->cartPos.x, this->cartPos.y, this->cartPos.z,
                    latitude, longitude, altitude);
                this->concentrationMsg.geo_point.latitude = latitude;
                this->concentrationMsg.geo_point.longitude = longitude;
                this->concentrationMsg.geo_point.altitude = altitude;
            }
        }
        else
        {
            this->concentrationMsg.geo_point.latitude = 0;
            this->concentrationMsg.geo_point.longitude = 0;
            this->concentrationMsg.geo_point.altitude = 0;
        }

        double totalParticleConc = 0.0;
        double smoothingParam;
        double particleConc;
        double distToParticle;

        double currentTime = ros::Time::now().toSec();

        double initSmoothingLength = std::pow(this->smoothingLength, 2.0 / 3);

        for (int i = 0; i < _msg->points.size(); i++)
        {
            // Compute the distance to the sensor
            distToParticle = std::sqrt(
                std::pow(_msg->points[i].x - this->cartPos.x, 2) +
                std::pow(_msg->points[i].y - this->cartPos.y, 2) +
                std::pow(_msg->points[i].z - this->cartPos.z, 2));

            smoothingParam = std::pow(initSmoothingLength +
                this->gamma * (currentTime - _msg->channels[0].values[i]), 1.5);
        }

        // Compute particle concentration
        if (distToParticle >= 0 && distToParticle < smoothingParam)
          particleConc = 4.0 -
            6.0 * std::pow(distToParticle / smoothingParam, 2) +
            3.0 * std::pow(distToParticle / smoothingParam, 3);
        else if (distToParticle >= smoothingParam && distToParticle < 2 * smoothingParam)
          particleConc = std::pow(2 - distToParticle / smoothingParam, 3);
        else
          particleConc = 0.0;

        particleConc *= 1 / (4 * M_PI * std::pow(smoothingParam, 3));
        totalParticleConc += particleConc;

        // Applying saturation
        this->concentrationMsg.concentration = std::min(
            this->gain * totalParticleConc, this->saturation);

        if (this->publishSalinity)
        {
            this->salinityMsg.header.frame_id = _msg->header.frame_id;
            this->salinityMsg.header.stamp = ros::Time::now();
            this->salinityMsg.position = this->concentrationMsg.position;
            this->salinityMsg.geo_point = this->concentrationMsg.geo_point;

            // Calculating salinity
            this->salinityMsg.salinity = this->referenceSalinityValue * \
                (this->saturation - this->concentrationMsg.concentration) + \
                this->concentrationMsg.concentration * this->plumeSalinityValue;

            // Adding noise to the salinity value
            this->salinityMsg.salinity += this->noiseAmp \
                * this->noiseModel(this->rndGen);
        }

        // Adding noise to concentration value
        this->concentrationMsg.concentration += this->noiseAmp \
            * this->noiseModel(this->rndGen);
        this->updateMeasurement = false;
    }
}

/////////////////////////////////////////////////
void CPCSensor::OnOdometryUpdate(
    const nav_msgs::Odometry::ConstPtr &_msg)
{
    if (this->concentrationPub.getNumSubscribers() > 0)
    {
        if (this->updateMeasurement)
            return;
        this->cartPos.x = _msg->pose.pose.position.x;
        this->cartPos.y = _msg->pose.pose.position.y;
        this->cartPos.z = _msg->pose.pose.position.z;
    }
}

/////////////////////////////////////////////////
void CPCSensor::OnGPSUpdate(
    const sensor_msgs::NavSatFix::ConstPtr &_msg)
{
    ROS_ASSERT(this->useGeoCoordinates);
    if (this->concentrationPub.getNumSubscribers() > 0)
    {
        if (this->updateMeasurement)
            return;
        this->geoPos.latitude = _msg->latitude;
        this->geoPos.longitude = _msg->longitude;
        this->geoPos.altitude = _msg->altitude;
        this->projection.Forward(
            _msg->latitude, _msg->longitude, _msg->altitude,
            this->cartPos.x, this->cartPos.y, this->cartPos.z);
    }
}

/////////////////////////////////////////////////
void CPCSensor::OnSensorUpdate(const ros::TimerEvent&)
{
    if (!this->areParticlesInit)
        return;
    this->concentrationPub.publish(this->concentrationMsg);
    if (this->publishSalinity)
        this->salinityPub.publish(this->salinityMsg);
}
