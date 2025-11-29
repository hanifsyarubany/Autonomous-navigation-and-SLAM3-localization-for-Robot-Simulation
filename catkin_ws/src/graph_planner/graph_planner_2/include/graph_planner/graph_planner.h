#ifndef GRAPH_PLANNER_H
#define GRAPH_PLANNER_H

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <utility>

// Add missing actionlib includes
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>

#define MAX_NODES 1000

struct Node {
    int id;
    int prev_id;
    double x, y;
    bool visited;
    bool mine;
};

struct Edge {
    double weight;
    int node;

    bool operator>(const Edge& other) const {
        return weight > other.weight;
    }
};

class GraphPlanner
{
public:
    ros::NodeHandle nh_;

    GraphPlanner(ros::NodeHandle& nh);
    ~GraphPlanner();
    
    double calculateDistance(double x1, double y1, double x2, double y2); 
    int findClosestNode(const geometry_msgs::PoseStamped& pose, const std::vector<Node>& nodes); 
    void addNode(int id, double x, double y, bool mine);
    void addEdge(int from, int to, double weight);
    void UpdateNode(int id, int prev_id, double x, double y, bool visited, bool mine);
    void UpdateEdgeWeight(int from, int to, double newWeight);
    void UpdateAllEdgeWeight();
    void PublishGraph();
    std::vector<Node> dijkstra(int start, int target);
    int findNearestMineNode(int start);

    void Plan();
    void LoadCSV(const std::string& filename); 
    void run();
    void sendNavigationGoal(double x, double y, double yaw);
    void sendNextGoal();

    int start_node = 0;
    int goal_node = 0;
    int index = 0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr p_nodes;
    sensor_msgs::PointCloud2 m_node_points;

    bool bGetRobotPose = false;
    bool bGetRvizGoal = false;
    bool bGetPath = false;
    bool bUseRvizGoal = true;
    bool bGetMineGoal = false;
    bool bPlanTrigger = true;
    bool bMineSearchTrigger = true;
    bool bGlobalPlanTrigger = false;
    bool bGlobalPlanReceived = false;
    bool bGlobalPlanDone = false;

    std::string csv_path, robot_postion_topic;

private:
    ros::Subscriber sub_robot_position; 
    ros::Subscriber sub_goal_position; 
    ros::Subscriber sub_global_plan_trigger; 
    ros::Subscriber sub_global_plan_received; 

    ros::Publisher pub_node_points, pub_robot_point, pub_goal_point, pub_path_nodes; 
    ros::Publisher pub_global_path;
    ros::Publisher node_pub;
    ros::Publisher node_text_pub;
    ros::Publisher edge_pub;
    ros::Publisher graph_trigger_pub;
    ros::Publisher new_graph_pub;

    // Action client for move_base - FIXED DECLARATION
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;
    ros::Publisher simple_goal_pub_;
    int current_goal_index_;
    bool use_actionlib_;  
    bool all_goals_completed_=false;

    void RobotPositionCB(const nav_msgs::Odometry &msg);
    void GoalPositionCB(const geometry_msgs::PoseStamped &msg);
    void GlobalPlanTriggerCB(const std_msgs::Bool &msg);
    void GlobalPlanReceivedCB(const std_msgs::Bool &msg);

    std::vector<std::vector<std::pair<int, double>>> graph;
    std::vector<Node> nodes;
    std::vector<Node> prev_solution;

    nav_msgs::Odometry m_odom;
    double ARRIVAL_THRES = 0.45;
    std_msgs::Bool GraphTrigger;
};

#endif