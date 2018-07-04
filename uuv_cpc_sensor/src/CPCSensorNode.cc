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

#include <ros/ros.h>
#include <uuv_cpc_sensor/CPCSensor.hh>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cpc_sensor");
    uuv_plume_simulator::CPCSensor sensor();

    // Spin
    ros::AsyncSpinner spinner(1); // Use n threads
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
