#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <first_project/Odom.h>
#include <first_project/ResetOdom.h>
#include <tf/transform_broadcaster.h>
#include <cmath>

double x_, y_, th_;
double wheelbase_ = 2.8;
ros::Time last_time_;

bool reset_cb(first_project::ResetOdom::Request&, first_project::ResetOdom::Response& res){
    x_ = y_ = th_ = 0.0;
    res.resetted = true;
    return true;
}

void cb(const geometry_msgs::Quaternion::ConstPtr& msg){
    double v = msg->x;
    double delta = msg->y;

    ros::Time now = ros::Time::now();
    double dt = (now - last_time_).toSec();
    if(dt <= 0 || dt > 1) dt = 0.02;

    double omega = v * std::tan(delta) / wheelbase_;

    x_ += v * std::cos(th_) * dt;
    y_ += v * std::sin(th_) * dt;
    th_ += omega * dt;

    static tf::TransformBroadcaster br;
    geometry_msgs::TransformStamped t;
    t.header.stamp = now;
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";
    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0;
    t.transform.rotation = tf::createQuaternionMsgFromYaw(th_);
    br.sendTransform(t);

    nav_msgs::Odometry od;
    od.header.stamp = now;
    od.header.frame_id = "odom";
    od.pose.pose.position.x = x_;
    od.pose.pose.position.y = y_;
    od.pose.pose.orientation = t.transform.rotation;
    od.child_frame_id = "base_link";
    od.twist.twist.linear.x = v;
    od.twist.twist.angular.z = omega;

    static ros::Publisher* od_pub_ptr = nullptr;
    static ros::Publisher* custom_pub_ptr = nullptr;

    if(!od_pub_ptr){
        ros::NodeHandle nh;
        od_pub_ptr = new ros::Publisher(nh.advertise<nav_msgs::Odometry>("odometry",10));
        custom_pub_ptr = new ros::Publisher(nh.advertise<first_project::Odom>("custom_odometry",10));
    }

    od_pub_ptr->publish(od);

    first_project::Odom custom;
    custom.x = x_;
    custom.y = y_;
    custom.th = th_;
    custom.timestamp = std::to_string(now.toSec());
    custom_pub_ptr->publish(custom);

    last_time_ = now;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "odom_node");
    ros::NodeHandle nh;

    nh.param("starting_x", x_, 0.0);
    nh.param("starting_y", y_, 0.0);
    nh.param("starting_th", th_, 0.0);
    nh.param("wheelbase", wheelbase_, 2.8);

    last_time_ = ros::Time::now();

    ros::ServiceServer srv = nh.advertiseService("reset_odom", reset_cb);
    ros::Subscriber sub = nh.subscribe("speed_steer", 10, cb);

    ros::spin();
    return 0;
}
