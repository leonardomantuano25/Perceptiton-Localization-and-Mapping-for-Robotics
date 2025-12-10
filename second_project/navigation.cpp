#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <fstream>
#include <vector>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct Waypoint {
    double x;
    double y;
    double theta;
};

class NavigationNode {
private:
    std::vector<Waypoint> waypoints;
    int current_waypoint;
    MoveBaseClient* ac;

public:
    NavigationNode(std::string filename) : current_waypoint(0) {
        loadWaypoints(filename);
        ac = new MoveBaseClient("move_base", true);
        
        while(!ac->waitForServer(ros::Duration(5.0))) {
            ROS_INFO("Waiting for move_base action server");
        }
    }

    ~NavigationNode() {
        delete ac;
    }

    void loadWaypoints(std::string filename) {
        std::ifstream file(filename);
        std::string line;
        
        while(std::getline(file, line)) {
            Waypoint wp;
            sscanf(line.c_str(), "%lf,%lf,%lf", &wp.x, &wp.y, &wp.theta);
            waypoints.push_back(wp);
        }
        
        ROS_INFO("Loaded %lu waypoints", waypoints.size());
    }

    void sendGoal(Waypoint wp) {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        
        goal.target_pose.pose.position.x = wp.x;
        goal.target_pose.pose.position.y = wp.y;
        
        tf::Quaternion q = tf::createQuaternionFromYaw(wp.theta);
        goal.target_pose.pose.orientation.x = q.x();
        goal.target_pose.pose.orientation.y = q.y();
        goal.target_pose.pose.orientation.z = q.z();
        goal.target_pose.pose.orientation.w = q.w();
        
        ROS_INFO("Sending goal %d: x=%.2f, y=%.2f, theta=%.2f", 
                 current_waypoint + 1, wp.x, wp.y, wp.theta);
        
        ac->sendGoal(goal, boost::bind(&NavigationNode::doneCb, this, _1, _2));
    }

    void doneCb(const actionlib::SimpleClientGoalState& state,
                const move_base_msgs::MoveBaseResultConstPtr& result) {
        if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Goal %d reached successfully", current_waypoint + 1);
            current_waypoint++;
            
            if(current_waypoint < waypoints.size()) {
                sendGoal(waypoints[current_waypoint]);
            } else {
                ROS_INFO("All waypoints reached!");
            }
        } else {
            ROS_WARN("Goal %d failed: %s", current_waypoint + 1, state.toString().c_str());
        }
    }

    void start() {
        if(waypoints.size() > 0) {
            sendGoal(waypoints[current_waypoint]);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "navigation");
    ros::NodeHandle nh("~");
    
    std::string waypoints_file;
    nh.param<std::string>("waypoints_file", waypoints_file, "waypoints.csv");
    
    NavigationNode nav(waypoints_file);
    nav.start();
    
    ros::spin();
    return 0;
}