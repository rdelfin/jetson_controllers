#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

geometry_msgs::TransformStamped get_transform(const std::string& parent_frame, const std::string& child_frame,
                                             double x, double y, double z, const tf2::Quaternion& q) {
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = parent_frame;
    transformStamped.child_frame_id = child_frame;
    transformStamped.transform.translation.x = x;
    transformStamped.transform.translation.y = y;
    transformStamped.transform.translation.z = z;
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    return transformStamped;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "base_laser_tf_node");
    ros::NodeHandle nh;

    ros::Rate r(10);

    tf2_ros::TransformBroadcaster br;

    while(ros::ok) {
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        br.sendTransform(get_transform("world", "base_link", 0, 0.05, 0, q));
        br.sendTransform(get_transform("base_link", "laser", 0, 0.11, 0, q));
        br.sendTransform(get_transform("map", "world", 0, 0, 0, q));

        r.sleep();
        ros::spinOnce();
    }
}