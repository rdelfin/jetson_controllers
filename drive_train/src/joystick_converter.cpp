#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#define SPEED_AXIS 2
#define Y_AXIS     1
#define X_AXIS     0

ros::Publisher cmd_vel_pub;

void joy_callback(const sensor_msgs::Joy::ConstPtr& msg_ptr) {
    double mult = (msg_ptr->axes[SPEED_AXIS] + 1.0) / 2.0;
    double x = msg_ptr->axes[X_AXIS];
    double y = msg_ptr->axes[Y_AXIS];

    geometry_msgs::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = y * mult;
    cmd_vel_msg.angular.z = x;
    cmd_vel_pub.publish(cmd_vel_msg);
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "joystick_converter");

    ros::NodeHandle nh;

    ros::Subscriber joy_sub = nh.subscribe("/joy", 1000, joy_callback);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    ros::spin();
}
