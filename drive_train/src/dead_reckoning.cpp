#include <ros/ros.h>
#include <jetson_control_msgs/PCA9685Command.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define MIN_PWM  204
#define ZERO_PWM 307
#define MAX_PWM  409

#define LENGTH 0.33

template<typename T>
inline T limit(T val, T min, T max) {
    if(val < min)
        return min;
    if(val > max)
        return max;
    return val;
}

double speed_factor, pos_speed_offset, neg_speed_offset, rot_factor;
int speed_port_num, steering_port_num;
uint32_t frame;
bool speed_set = false, steering_set = false;
double speed, direction;
geometry_msgs::Point pos;
double rot;

// Update commands
void command_callback(const jetson_control_msgs::PCA9685Command::ConstPtr& msg) {
    int port = msg->pwm_port;

    if(port == speed_port_num) {
        speed_set = true;

        // Speed value between 0 and 1
        double normalized_val = ((double)limit(msg->value, MIN_PWM, MAX_PWM) - (double)ZERO_PWM)/(double)(MAX_PWM - ZERO_PWM);

        if(normalized_val >= 0) {
            speed = normalized_val*speed_factor - pos_speed_offset;
            if(speed < 0)
                speed = 0;
        }
        else {
            speed = normalized_val*speed_factor + neg_speed_offset;
            if(speed > 0)
                speed = 0;
        }
    } else if(port == steering_port_num) {
        steering_set = true;

        direction = msg->value*rot_factor;
    }
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "dead_reckoning");
    ros::NodeHandle nh;


    nh.param<double>("speed_factor", speed_factor, 1);
    nh.param<double>("pos_speed_offset", pos_speed_offset, 0);
    nh.param<double>("neg_speed_offset", neg_speed_offset, 0);
    nh.param<double>("rot_factor", rot_factor, 0);

    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    ros::Subscriber sub = nh.subscribe("pca9685_commands", 50, command_callback);
    tf2_ros::TransformBroadcaster odom_broadcaster;

    ros::Rate r(10);

    ros::Time prev_time = ros::Time();

    while(ros::ok) {
        ros::Time curr_time = ros::Time();
        if(speed_set && steering_set) {
            double dxdt = speed*(curr_time - prev_time).toNSec()/1000000000.0;
            pos.x += std::cos(rot)*dxdt;
            pos.y += std::sin(rot)*dxdt;
            rot += std::tan(direction)/LENGTH * dxdt;

            nav_msgs::Odometry odom_msg;
            geometry_msgs::TransformStamped transformStamped;

            transformStamped.header.stamp = curr_time;
            transformStamped.header.frame_id = "odom";
            transformStamped.child_frame_id = "base_link";
            transformStamped.transform.translation.x = pos.x;
            transformStamped.transform.translation.y = pos.y;
            transformStamped.transform.translation.z = pos.z;
            transformStamped.transform.rotation = tf2::toMsg(tf2::Quaternion());

            odom_broadcaster.sendTransform(transformStamped);

            odom_msg.header.seq = frame++;
            odom_msg.header.stamp = curr_time;
            odom_msg.header.frame_id = "odom";
            odom_msg.child_frame_id = "base_link";

            odom_msg.pose.pose.position = pos;
            odom_msg.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(rot, 0, 0));

            odom_msg.twist.twist.linear.x = std::cos(rot)*speed;
            odom_msg.twist.twist.linear.y = std::sin(rot)*speed;
            odom_msg.twist.twist.angular.z = direction;
            

            odom_pub.publish(odom_msg);
        }

        r.sleep();
        ros::spinOnce();
        prev_time = curr_time;
    }
}