#include <ros/ros.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>
#include <climits>
#include <queue>
#include <iostream>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>

#include <graph_planner/graph_planner.h>
#include <tf/tf.h>



GraphPlanner::GraphPlanner(ros::NodeHandle &nh) : 
    nh_(nh), 
    ac_("move_base", true),  // Action client initialization
    current_goal_index_(1),
    use_actionlib_(false),
    all_goals_completed_(false)  // Initialize all members
{
    nh_.getParam("/graph_planner_node/csv_path", csv_path);

    // Initialize publishers
    pub_global_path = nh_.advertise<nav_msgs::Path>("/graph_planner/path/global_path", 1);
    simple_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    
    // Wait for the action server to come up
    ROS_INFO("Waiting for move_base action server...");
    if(ac_.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Connected to move_base action server");
        use_actionlib_ = true;
    } else {
        ROS_WARN("Could not connect to move_base action server, using simple goal publisher");
        use_actionlib_ = false;
    }
    
    LoadCSV(csv_path);
}
GraphPlanner::~GraphPlanner()
{
}

void GraphPlanner::addNode(int id, double x, double y, bool mine)
{
    nodes.push_back({id, 0, x, y, false, mine});
    graph.resize(nodes.size());
}

void GraphPlanner::addEdge(int from, int to, double weight)
{
    graph[from - 1].emplace_back(to, weight); // Adjusting to 0-based index
}

void GraphPlanner::PublishGraph()
{
    nav_msgs::Path output;
    output.header.frame_id = "odom";

    geometry_msgs::PoseStamped pt;
    double previous_x = 0.0;
    double previous_y = 0.0;

    for (size_t i = 0; i < nodes.size(); ++i)
    {
        const auto& node = nodes[i];

        pt.pose.position.x = node.x;
        pt.pose.position.y = node.y;
        pt.pose.position.z = 0.0;

        if (i == 0)
        {
            // 첫 번째 값은 회전 값을 고정
            pt.pose.orientation.w = 1.0;
            pt.pose.orientation.x = 0.0;
            pt.pose.orientation.y = 0.0;
            pt.pose.orientation.z = 0.0;
        }
        else
        {
            // 이전 값과 현재 값 사이의 방향을 계산하여 heading 계산
            double delta_x = node.x - previous_x;
            double delta_y = node.y - previous_y;
            double heading = atan2(delta_y, delta_x);

            // heading을 Quaternion으로 변환
            tf::Quaternion q = tf::createQuaternionFromYaw(heading);

            // Quaternion 값을 geometry_msgs::Pose에 저장
            pt.pose.orientation.w = q.getW();
            pt.pose.orientation.x = q.getX();
            pt.pose.orientation.y = q.getY();
            pt.pose.orientation.z = q.getZ();
        }

        output.poses.push_back(pt);

        // 이전 값을 업데이트
        previous_x = node.x;
        previous_y = node.y;
    }

    ROS_INFO("Publish Global Path");
    pub_global_path.publish(output);
}

void GraphPlanner::LoadCSV(const std::string &filename)
{
    std::cout << filename << std::endl;
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Check file path." << std::endl;
    }

    std::string line;
    std::string header;

    getline(file, header);
    while (getline(file, line))
    {
        std::stringstream ss(line);
        std::string id, x, y, mine, edgeStr;

        getline(ss, id, ' ');
        getline(ss, x, ' ');
        getline(ss, y, ' ');
        getline(ss, mine, ' ');
        getline(ss, edgeStr, ' ');

        int nodeId = std::stoi(id);
        double nodeX = std::stod(x);
        double nodeY = std::stod(y);
        bool mine_check = false;
        // bool mine
        if (mine == "True")
        {
            mine_check = true;
        }

        addNode(nodeId, nodeX, nodeY, mine_check);

        // Process the edge information
        edgeStr.erase(std::remove(edgeStr.begin(), edgeStr.end(), '['), edgeStr.end());
        edgeStr.erase(std::remove(edgeStr.begin(), edgeStr.end(), ']'), edgeStr.end());

        std::stringstream edgeStream(edgeStr);
        std::vector<int> edges;
        std::string edgeValue;

        while (getline(edgeStream, edgeValue, ','))
        {
            edges.push_back(std::stoi(edgeValue));
        }
        for (int neighborId : edges)
        {
            addEdge(nodeId, neighborId, 0.0);
        }
    }
    file.close();
}

void GraphPlanner::sendNavigationGoal(double x, double y, double yaw) {
    if(use_actionlib_ && ac_.isServerConnected()) {
        // Method 1: Using actionlib (recommended for reliable navigation)
        move_base_msgs::MoveBaseGoal goal;
        
        goal.target_pose.header.frame_id = "odom";
        goal.target_pose.header.stamp = ros::Time::now();
        
        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y;
        goal.target_pose.pose.position.z = 0.0;
        
        // Convert yaw to quaternion
        goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        
        ac_.sendGoal(goal);
        ROS_INFO("Sent navigation goal via actionlib: (%.2f, %.2f)", x, y);
        
        // Optional: Wait for result with timeout
        bool finished = ac_.waitForResult(ros::Duration(30.0));
        if(finished) {
            actionlib::SimpleClientGoalState state = ac_.getState();
            ROS_INFO("Action finished: %s", state.toString().c_str());
        } else {
            ROS_WARN("Action did not finish before timeout.");
        }
    } else {
        // Method 2: Using simple publisher (good for testing)
        geometry_msgs::PoseStamped goal;
        
        goal.header.frame_id = "odom";
        goal.header.stamp = ros::Time::now();
        
        goal.pose.position.x = x;
        goal.pose.position.y = y;
        goal.pose.position.z = 0.0;
        goal.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        
        simple_goal_pub_.publish(goal);
        ROS_INFO("Published simple goal to: (%.2f, %.2f)", x, y);
        ros::WallDuration(100).sleep();
    }
}


void GraphPlanner::sendNextGoal() {
    if(current_goal_index_ < nodes.size()) {
        auto& node = nodes[current_goal_index_];
        
        // Calculate orientation towards next point (if available)
        double yaw = 0.0;
        if(current_goal_index_ + 1 < nodes.size()) {
            auto& next_node = nodes[current_goal_index_ + 1];
            double dx = next_node.x - node.x;
            double dy = next_node.y - node.y;
            yaw = atan2(dy, dx);
        }
       
        sendNavigationGoal(node.x, node.y, yaw);
        current_goal_index_++;
    } else {
        ROS_INFO("All goals completed!");
        all_goals_completed_ = true;
    }
}
void GraphPlanner::run()
{
    PublishGraph();
    
    static bool goals_sent = false;
    while (!all_goals_completed_) {
        // Send first node as goal
        sendNextGoal();
        goals_sent = true;
        ROS_INFO("Sent initial navigation goal");
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "GraphPlanner");
    ros::NodeHandle _nh("~");

    printf("Initiate: GraphPlanner\n");
    GraphPlanner node_link_loader(_nh);
    node_link_loader.run();
    printf("Terminate: GraphPlanner\n");

    return 0;
}