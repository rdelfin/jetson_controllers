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
#include <jetson_control_msgs/PCA9685Command.h>

#include <tuple>
#include <unordered_map>

std::unordered_map<int, PCA9685*> connections;

void command_callback(const jetson_control_msgs::PCA9685Command::ConstPtr& msg) {
    int addr = msg->serial_address;

    if(connections.count(addr) == 0) {
        connections[addr] = new PCA9685(addr);
        if(!connections[addr]->openPCA9685()) {
            ROS_WARN("Error openning PCA9685 serial port %x. Ignoring...", addr);
            delete connections[addr];
            connections.erase(addr);
            return;
        }
        connections[addr]->setPWMFrequency(50);
    }

    connections[addr]->setPWM(msg->pwm_port, 0, msg->value);
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "pca9685_driver");

    ros::NodeHandle nh;
    ros::Subscriber command_sub = nh.subscribe("pca9685_commands", 100, command_callback);

    ros::Rate r(30);
    while(ros::ok()) {
        r.sleep();
        ros::spinOnce();
    }

    for(std::pair<int, PCA9685*> con_pair : connections) {
        con_pair.second->closePCA9685();
        delete con_pair.second;
    }
}