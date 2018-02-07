// Copyright 2018 Ricardo Delfin Garcia

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <ros/ros.h>

#include <pca9685_driver/JHPWMPCA9685.h>

int main(int argc, char* argv[]) {
    PCA9685 pca9685(0x44);
    if(pca9685.openPCA9685()) {
        ROS_ERROR("Error openning PCA9685.");
        exit(1);
    }

    ros::init(argc, argv, "pca9685_driver");

    ros::Rate r(30);
    while(ros::ok()) {
        r.sleep();
        ros::spinOnce();
    }

    pca9685.closePCA9685();
}