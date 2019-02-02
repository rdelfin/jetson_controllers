#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#define LASER_ANGLE_OFFSET -1.67

double distance;
bool laser_set = false;

enum State {
    STATE_INIT,
    STATE_SPEED_UP,
    STATE_HOLD,
    STATE_STOPPED
};

State state = STATE_SPEED_UP;

void laser_cb(const sensor_msgs::LaserScan::ConstPtr& laser_msg) {
    double laser_angle = LASER_ANGLE_OFFSET + laser_msg->angle_min;
    if(laser_angle < laser_msg->angle_min)
        laser_angle = laser_msg->angle_max + laser_angle - laser_msg->angle_min;

    size_t laser_idx = std::max((laser_angle+laser_msg->angle_min) / laser_msg->angle_increment, 0.0) + 1;
    long offset = 0;
    double test_dist = laser_msg->ranges[laser_idx];

    while(std::isinf(test_dist)) {
        if(offset > 0)
            offset = -offset;
        else
            offset = -offset + 1;
        test_dist = laser_msg->ranges[laser_idx+offset];
    }

    distance = test_dist;

    laser_set = true;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "speed_estimator");
    ros::NodeHandle nh("~");

    ros::Publisher speed_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 50);
    ros::Subscriber laser_sub = nh.subscribe("/scan", 50, laser_cb);

    double start_distance = 0, end_distance = 0;
    double speed = nh.param<double>("speed", 0);

    ros::Rate r(30);
    ros::Time timer_start;

    while(ros::ok()) {
        if(laser_set) {
            geometry_msgs::Twist speed_msg;
            speed_msg.angular.z = -0.12;
            if(state == STATE_INIT) {
                speed_msg.linear.x = speed;
                state = STATE_SPEED_UP;
                timer_start = ros::Time::now();
            }

            else if(state == STATE_SPEED_UP) {
                speed_msg.linear.x = speed;
                if((ros::Time::now() - timer_start) > ros::Duration(0.5)) {
                    state = STATE_HOLD;
                    start_distance = distance;
                    timer_start = ros::Time::now();
                }
            }

            else if(state == STATE_HOLD) {
                speed_msg.linear.x = speed;
                ros::Time now_time = ros::Time::now();

                if((now_time - timer_start) > ros::Duration(1.5)) {
                    end_distance = distance;
                    double travel_time = (now_time - timer_start).toSec();
                    double travel_dist = std::abs(end_distance - start_distance);
                    ROS_INFO("Travel information:");
                    ROS_INFO("\tTravel time: %f", travel_time);
                    ROS_INFO("\tDistance traveled: %f", travel_dist);
                    ROS_INFO("\tSpeed: %f", travel_dist/travel_time);
                    state = STATE_STOPPED;
                }
            }

            else if(state == STATE_STOPPED)
                speed_msg.linear.x = 0;

            else
                speed_msg.linear.x = 0;

            //ROS_INFO("Speed msg: %s", speed_msg);
            speed_pub.publish(speed_msg);
        }

        r.sleep();
        ros::spinOnce();
    }
}
