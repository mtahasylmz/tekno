#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "static_transform_publisher_node");
    ros::NodeHandle nh;
    
    tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0.0, -0.13, 0.018));
    tf::Quaternion q;
    q.setRPY(0, 0, 0); // Set roll, pitch, yaw
    transform.setRotation(q);

    ros::Rate rate(100); // Publish at 100 Hz

    while (ros::ok()) {
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "right_wheel_link"));
        rate.sleep();
    }

    return 0;
}