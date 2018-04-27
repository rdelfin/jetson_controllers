#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <jetson_control_msgs/PCA9685Command.h>

#define SERIAL_PORT       0x44
#define STEERING_PWM_PORT 0
#define SPEED_PWM_PORT    1

#define ZERO_PWM 307
#define MAX_PWM  409

#define BOUNDS(val, min, max) ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))

ros::Publisher pca9685_publisher;

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    ROS_INFO("RECEIVED MESSAGE");
    double lin = (MAX_PWM-ZERO_PWM)*BOUNDS(msg->linear.x, 0.0, 1.0) + ZERO_PWM;
    double rot = (MAX_PWM-ZERO_PWM)*BOUNDS(-msg->angular.z,-1.0, 1.0) + ZERO_PWM; // This one's flipped for whatever reason

    jetson_control_msgs::PCA9685Command steering_msg, speed_msg;
    steering_msg.serial_address = speed_msg.serial_address = SERIAL_PORT;
    steering_msg.pwm_port = STEERING_PWM_PORT;
    speed_msg.pwm_port = SPEED_PWM_PORT;
    steering_msg.value = rot;
    speed_msg.value = lin;

    pca9685_publisher.publish(steering_msg);
    pca9685_publisher.publish(speed_msg);
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "drive_train_controller");

    ros::NodeHandle nh;
    pca9685_publisher = nh.advertise<jetson_control_msgs::PCA9685Command>("/pca9685_commands", 1000);
    ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 1000, cmdVelCallback);

    ros::spin();
}
