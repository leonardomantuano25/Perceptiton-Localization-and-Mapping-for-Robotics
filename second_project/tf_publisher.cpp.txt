#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

class TFPublisher {
private:
    ros::NodeHandle nh;
    ros::Subscriber odom_sub;
    tf::TransformBroadcaster br;

public:
    TFPublisher() {
        odom_sub = nh.subscribe("/t265/odom", 10, &TFPublisher::odomCallback, this);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, 
                                        msg->pose.pose.position.y, 
                                        msg->pose.pose.position.z));
        
        tf::Quaternion q(msg->pose.pose.orientation.x,
                        msg->pose.pose.orientation.y,
                        msg->pose.pose.orientation.z,
                        msg->pose.pose.orientation.w);
        transform.setRotation(q);

        br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, 
                                             "odom", "base_link"));
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_publisher");
    TFPublisher tf_pub;
    ros::spin();
    return 0;
}