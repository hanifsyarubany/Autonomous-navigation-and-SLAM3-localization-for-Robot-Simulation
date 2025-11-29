#include "control_space_planner/control_space_planner_node.hpp"

/* ----- Class Functions ----- */
MotionPlanner::MotionPlanner(ros::NodeHandle& nh) : nh_(nh)
{
  // Subscriber
  subOccupancyGrid = nh.subscribe("/map/local_map/obstacle",1, &MotionPlanner::CallbackOccupancyGrid, this);
  subEgoOdom = nh.subscribe("/odom",1, &MotionPlanner::CallbackEgoOdom, this);
  subGoalPoint = nh.subscribe("/move_base_simple/goal",1, &MotionPlanner::CallbackGoalPoint, this);
  subGlobalPath = nh.subscribe("/graph_planner/path/global_path", 1, &MotionPlanner::CallbackGlobalPath, this);

  // Publisher
  pubSelectedMotion = nh_.advertise<sensor_msgs::PointCloud2>("/points/selected_motion", 1, true);
  pubMotionPrimitives = nh_.advertise<sensor_msgs::PointCloud2>("/points/motion_primitives", 1, true);
  pubCommand = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
  pubTruncTarget = nh_.advertise<geometry_msgs::PoseStamped>("/car/trunc_target", 1, true);
  
};

MotionPlanner::~MotionPlanner() 
{    
    ROS_INFO("MotionPlanner destructor.");
}

/* ----- ROS Functions ----- */

void MotionPlanner::CallbackOccupancyGrid(const nav_msgs::OccupancyGrid& msg)
{
  this->localMap = msg;
  this->origin_x = msg.info.origin.position.x;
  this->origin_y = msg.info.origin.position.y;
  this->frame_id = msg.header.frame_id;
  this->mapResol = msg.info.resolution;
  bGetMap = true;
  ROS_INFO_STREAM_THROTTLE(1.0, "[Map] frame=" << msg.header.frame_id
  << " res=" << msg.info.resolution
  << " size=" << msg.info.width << "x" << msg.info.height
  << " origin=(" << msg.info.origin.position.x << ","
                 << msg.info.origin.position.y << ")");
}

void MotionPlanner::CallbackGoalPoint(const geometry_msgs::PoseStamped& msg)
{
  this->goalPose = msg;
  // - position
  this->goal_x = msg.pose.position.x;
  this->goal_y = msg.pose.position.y;
  // - orientation
  // -- quaternion to RPY (global)
  tf2::Quaternion goal;
  double goal_roll, goal_pitch, goal_yaw;
  // --- copy quaternion from odom
  tf2::convert(msg.pose.orientation, goal);
  // --- get roll pitch yaw
  tf2::Matrix3x3 m_goal(goal);
  m_goal.getRPY(goal_roll, goal_pitch, goal_yaw);
  this->goal_yaw = goal_yaw;
  
  this->bGetGoal = true;
}

// Modify CallbackGlobalPath:
void MotionPlanner::CallbackGlobalPath(const nav_msgs::Path& msg)
{
    if (msg.poses.empty()) return;

    // Store the static path if this is the first time or path changed
    if (!path_initialized_ || msg.poses.size() != static_global_path_.size()) {
        static_global_path_ = msg.poses;
        current_path_index_ = 0;
        path_initialized_ = true;
        ROS_WARN_STREAM("[GlobalPath] Stored static path with " << static_global_path_.size() << " waypoints");
    }

    // Get current robot position
    double rx = ego_x, ry = ego_y;

    // Find the closest waypoint ahead of us
    int best_index = current_path_index_;
    double best_progress = -1e9;

    for (int i = current_path_index_; i < static_global_path_.size(); i++) {
        const auto& p = static_global_path_[i];
        double dx = p.pose.position.x - rx;
        double dy = p.pose.position.y - ry;
        double dist = std::sqrt(dx*dx + dy*dy);

        // Use robot's current heading to determine "ahead"
        double s_forward = std::cos(ego_yaw)*dx + std::sin(ego_yaw)*dy;

        if (s_forward > 0.1) { // At least slightly ahead
            if (s_forward > best_progress) {
                best_progress = s_forward;
                best_index = i;
            }
        }
    }

    // If we're close to current target, move to next waypoint
    double current_dx = static_global_path_[current_path_index_].pose.position.x - rx;
    double current_dy = static_global_path_[current_path_index_].pose.position.y - ry;
    double current_dist = std::sqrt(current_dx*current_dx + current_dy*current_dy);

    if (current_dist < 1.0 && current_path_index_ < static_global_path_.size() - 1) {
        current_path_index_++;
        ROS_WARN_STREAM("[Path] Advancing to waypoint " << current_path_index_);
    }

    // Use the selected waypoint
    const auto& pick = static_global_path_[current_path_index_];
    this->goalPose = pick;
    this->goal_x   = pick.pose.position.x;
    this->goal_y   = pick.pose.position.y;

    tf2::Quaternion q; double r,p,y;
    tf2::fromMsg(pick.pose.orientation, q);
    tf2::Matrix3x3(q).getRPY(r,p,y);
    this->goal_yaw = y;

    this->bGetGoal = true;

    ROS_INFO_STREAM_THROTTLE(1.0, "[Path->Goal] index=" << current_path_index_ 
                            << "/" << static_global_path_.size()
                            << " at (" << goal_x << "," << goal_y << ")"
                            << " dist=" << current_dist << "m");
}

void MotionPlanner::CallbackEgoOdom(const nav_msgs::Odometry& msg)
{
  this->egoOdom = msg;
  // - position
  this->ego_x = msg.pose.pose.position.x;
  this->ego_y = msg.pose.pose.position.y;
  // - orientation
  // -- quaternion to RPY (global)
  tf2::Quaternion ego;
  double ego_roll, ego_pitch, ego_yaw;
  // --- copy quaternion from odom
  tf2::convert(msg.pose.pose.orientation, ego);
  // --- get roll pitch yaw
  tf2::Matrix3x3 m_ego(ego);
  m_ego.getRPY(ego_roll, ego_pitch, ego_yaw);
  this->ego_yaw = ego_yaw;
  
  this->bGetEgoOdom = true;
}

void MotionPlanner::PublishSelectedMotion(std::vector<Node> motionMinCost)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_ptr(new pcl::PointCloud<pcl::PointXYZI>);

  // publish selected motion primitive as point cloud
  for (auto motion : motionMinCost) {
    pcl::PointXYZI pointTmp;
    pointTmp.x = motion.x;
    pointTmp.y = motion.y;
    cloud_in_ptr->points.push_back(pointTmp);
  }

  sensor_msgs::PointCloud2 motionCloudMsg;
  pcl::toROSMsg(*cloud_in_ptr, motionCloudMsg);
  motionCloudMsg.header.frame_id = this->frame_id;
  motionCloudMsg.header.stamp = ros::Time::now();
  pubSelectedMotion.publish(motionCloudMsg);
}

void MotionPlanner::PublishMotionPrimitives(std::vector<std::vector<Node>> motionPrimitives)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_ptr(new pcl::PointCloud<pcl::PointXYZI>);

  // publish motion primitives as point cloud
  for (auto& motionPrimitive : motionPrimitives) {
    if (motionPrimitive.empty()) continue;
    double cost_total = motionPrimitive.back().cost_total;
    for (auto motion : motionPrimitive) {
      pcl::PointXYZI pointTmp;
      pointTmp.x = motion.x;
      pointTmp.y = motion.y;
      pointTmp.z = cost_total;
      pointTmp.intensity = cost_total;
      cloud_in_ptr->points.push_back(pointTmp);
    }
  }
  
  sensor_msgs::PointCloud2 motionPrimitivesCloudMsg;
  pcl::toROSMsg(*cloud_in_ptr, motionPrimitivesCloudMsg);
  motionPrimitivesCloudMsg.header.frame_id = this->frame_id;
  motionPrimitivesCloudMsg.header.stamp = ros::Time::now();
  pubMotionPrimitives.publish(motionPrimitivesCloudMsg);
}

void MotionPlanner::PublishCommand(std::vector<Node> motionMinCost)
{
  if (motionMinCost.empty()) return;  
  geometry_msgs::Twist command;
  // low-level control
  double steering_angle = motionMinCost.back().delta;
  double yaw = normalizePiToPi(motionMinCost.back().yaw);
  double speed_norm = 0.1 * this->MOTION_VEL * motionMinCost.size() / (this->MAX_PROGRESS / this->DIST_RESOL);
  
  command.angular.z = speed_norm * tan(steering_angle) / this->WHEELBASE * 0.5;
  command.linear.x  = speed_norm * cos(yaw);
  command.linear.y  = speed_norm * sin(yaw);

  // arrival rule
  if (bGetGoal && bGetLocalNode) {
    double distToGoal = std::sqrt(this->localNode.x*this->localNode.x + this->localNode.y*this->localNode.y);

    if (distToGoal < this->ARRIVAL_THRES) {
      command.angular.z = 0.0;
      command.linear.x = 0.0;
    }
    else if (distToGoal < 2*this->ARRIVAL_THRES) {
      // command.linear.x = command.linear.x * pow(distToGoal / 2*this->ARRIVAL_THRES, 3.0);
      command.linear.x *= std::pow(distToGoal / (2 * this->ARRIVAL_THRES), 3.0);
    }
  }

  pubCommand.publish(command);
}

/* ----- Algorithm Functions ----- */

void MotionPlanner::Plan()
{
  // Compute current LOS target pose
  if (this->bGetEgoOdom && this->bGetGoal) {
    Node goalNode;
    goalNode.x = this->goal_x;
    goalNode.y = this->goal_y;
    goalNode.yaw = this->goal_yaw;
    localNode = GlobalToLocalCoordinate(goalNode, this->egoOdom);
    // - compute truncated local node pose within local map
    Node tmpLocalNode;
    memcpy(&tmpLocalNode, &localNode, sizeof(struct Node));
    tmpLocalNode.x = std::max(this->mapMinX, std::min(tmpLocalNode.x, this->mapMaxX));
    tmpLocalNode.y = std::max(this->mapMinY, std::min(tmpLocalNode.y, this->mapMaxY));
    truncLocalNode = tmpLocalNode;

    // for debug
    geometry_msgs::PoseStamped localPose = GlobalToLocalCoordinate(this->goalPose, this->egoOdom);
    localPose.header.frame_id = "base_link";
    pubTruncTarget.publish(localPose);

    this->bGetLocalNode = true;

    ROS_INFO_STREAM_THROTTLE(1.0, "[GoalLocal] (x,y,yaw)=("
    << localNode.x << "," << localNode.y << "," << localNode.yaw
    << ")  [Trunc] (" << truncLocalNode.x << "," << truncLocalNode.y << ")");
  }

  // Motion generation
  motionCandidates = GenerateMotionPrimitives(this->localMap);
  
  // Select motion
  std::vector<Node> motionMinCost = SelectMotion(motionCandidates);

  // Publish data
  PublishData(motionMinCost, motionCandidates);
}

std::vector<std::vector<Node>> MotionPlanner::GenerateMotionPrimitives(nav_msgs::OccupancyGrid localMap)
{
  /*
    TODO: Generate motion primitives
    - you can change the below process if you need.
    - you can calculate cost of each motion if you need.
  */
  // initialize motion primitives
  std::vector<std::vector<Node>> motionPrimitives;
  // compute params w.r.t. uncertainty
  int num_candidates = static_cast<int>(this->MAX_DELTA * 2 / this->DELTA_RESOL) + 1;
  // For mecanum, generate motions with different heading changes
  for (int i = 0; i < num_candidates; i++) {
    // current steering delta
    double angle_delta = this->MAX_DELTA - i * this->DELTA_RESOL;
    // Scale delta to be more appropriate for mecanum (smaller values)
    double scaled_delta = angle_delta * 0.3;
    // <<MODIFICATION>> seed a simple steering-control cost that depends only on |delta|
    const double cost_control = std::abs(scaled_delta);
    // <<MODIFICATION>> init start node
    Node startNode(0, 0, 0, 0, scaled_delta, cost_control, 0, 0, -1, false);
    // rollout to generate motion
    std::vector<Node> motionPrimitive = RolloutMotion(startNode, this->MAX_PROGRESS, localMap);
    // add current motionPrimitive
    motionPrimitives.push_back(motionPrimitive);
  }
  // Stream
  ROS_INFO_STREAM_THROTTLE(1.0, "[Gen] num_primitives=" << motionPrimitives.size());
  return motionPrimitives;
}

std::vector<Node> MotionPlanner::RolloutMotion(Node startNode,
                                              double maxProgress,
                                              nav_msgs::OccupancyGrid localMap)
{
  /*
    TODO: rollout to generate a motion primitive based on the current steering angle
    - calculate cost terms here if you need
    - check collision / sensor range if you need
    1. Update motion node using current steering angle delta based on the vehicle kinematics equation.
    2. collision checking
    3. range checking
  */

  // Initialize motionPrimitive
  std::vector<Node> motionPrimitive;

  // Check collision and compute traversability cost for each motion node of primitive (in planner coordinate)
  Node currMotionNode(startNode.x, startNode.y, 0, 0, startNode.delta, 0, 0, 0, -1, false);
  double progress = this->DIST_RESOL;

  // for compute closest distance toward goal point. You can use in SelectMotion function to calculate goal distance cost
  double minDistGoal = 987654321;
  if (this->bGetLocalNode) {
    minDistGoal = std::sqrt((startNode.x-truncLocalNode.x)*(startNode.x-truncLocalNode.x) +
                       (startNode.y-truncLocalNode.y)*(startNode.y-truncLocalNode.y));
  }

  //! 1. Update motion node using current steering angle delta based on the vehicle kinematics equation
  // - while loop until maximum progress of a motion

  // Loop for rollout
  while (progress < maxProgress) {
    // x_t+1   := x_t + x_dot * dt
    // y_t+1   := y_t + y_dot * dt
    // yaw_t+1 := yaw_t + yaw_dot * dt
    // <<MODIFICATION>>
    currMotionNode.x += this->DIST_RESOL * cos(currMotionNode.yaw);
    currMotionNode.y += this->DIST_RESOL * sin(currMotionNode.yaw);
    currMotionNode.yaw = normalizePiToPi(currMotionNode.yaw + currMotionNode.delta * this->TIME_RESOL);

    // Calculate minimum distance toward goal
    if (this->bGetLocalNode) {
      double distGoal = std::sqrt((currMotionNode.x-truncLocalNode.x)*(currMotionNode.x-truncLocalNode.x) +
                             (currMotionNode.y-truncLocalNode.y)*(currMotionNode.y-truncLocalNode.y));
      if (minDistGoal > distGoal) {
        minDistGoal = distGoal;
      }
    }
    currMotionNode.minDistGoal = minDistGoal; // save current minDistGoal at current node
    
    // collision/range chekcing with "lookahead" concept
    // - lookahead point
    // double aheadYaw = currMotionNode.yaw + this->MOTION_VEL * tan(startNode.delta) / this->WHEELBASE * this->TIME_RESOL;
    double aheadX = currMotionNode.x + this->LOOKAHEAD_DIST * cos(currMotionNode.yaw);
    double aheadY = currMotionNode.y + this->LOOKAHEAD_DIST * sin(currMotionNode.yaw);

    //! 2. collision checking
    // - local to map coordinate transform
    // Node collisionPointNode(currMotionNode.x, currMotionNode.y, currMotionNode.z, currMotionNode.yaw, currMotionNode.delta, 0, 0, 0, -1, false);
    Node collisionPointNode(aheadX, aheadY, currMotionNode.z, currMotionNode.yaw, 
                           currMotionNode.delta, 0, 0, 0, -1, false);
    Node collisionPointNodeMap = LocalToPlannerCorrdinate(collisionPointNode);
    // ROS_INFO_THROTTLE(1.0, "[Map] frame=%s res=%.3f size=%ux%u origin=(%.2f,%.2f)",
    //               this->frame_id.c_str(), this->mapResol,
    //               this->localMap.info.width, this->localMap.info.height,
    //               this->origin_x, this->origin_y);
    if (CheckCollision(collisionPointNodeMap, localMap)) {
      // - do some process when collision occurs.
      // - you can save collision information & calculate collision cost here.
      // - you can break and return current motion primitive or keep generate rollout.
      // <<MODIFICATION>>
      currMotionNode.cost_colli = 1e4;
      currMotionNode.collision = true;
      motionPrimitive.push_back(currMotionNode);
      progress += this->DIST_RESOL;
      ROS_WARN_STREAM_THROTTLE(1.0, "[Rollout] COLLISION at (grid) "
      << collisionPointNodeMap.x << "," << collisionPointNodeMap.y
      << "  prim_len=" << motionPrimitive.size());

      return motionPrimitive;
    }

    //! 3. range checking
    // - if you want to filter out motion points out of the sensor range, calculate the Line-Of-Sight (LOS) distance & yaw angle of the node
    // - LOS distance := sqrt(x^2 + y^2)
    // - LOS yaw := atan2(y, x)
    // - if LOS distance > MAX_SENSOR_RANGE or abs(LOS_yaw) > FOV*0.5 <-- outside of sensor range 
    // - if LOS distance <= MAX_SENSOR_RANGE and abs(LOS_yaw) <= FOV*0.5 <-- inside of sensor range
    // - use params in header file (MAX_SENSOR_RANGE, FOV)
    // <<MODIFICATION>>
    double LOS_DIST = std::sqrt((currMotionNode.x)*(currMotionNode.x)+(currMotionNode.y)*(currMotionNode.y));
    double LOS_YAW = std::atan2(currMotionNode.y,currMotionNode.x);
    if (LOS_DIST > this->MAX_SENSOR_RANGE || std::abs(LOS_YAW) > (this->FOV) * 0.5) {
      // -- do some process when out-of-range occurs.
      // -- you can break and return current motion primitive or keep generate rollout.
      ROS_INFO_STREAM_THROTTLE(1.0, "[Rollout] RANGE limit: LOS_DIST=" << LOS_DIST
      << " FOVcheck=" << std::abs(LOS_YAW) << " > " << (this->FOV*0.5)
      << " prim_len=" << motionPrimitive.size());
      return motionPrimitive;
    } 

    // append collision-free motion in the current motionPrimitive
    motionPrimitive.push_back(currMotionNode);

    // update progress of motion
    progress += this->DIST_RESOL;
  }
  ROS_DEBUG_STREAM_THROTTLE(0.5, "[Rollout] sz=" << motionPrimitive.size()
    << " x=" << currMotionNode.x << " y=" << currMotionNode.y
    << " yaw=" << currMotionNode.yaw << " delta=" << currMotionNode.delta);
  // return current motion
  return motionPrimitive;
}


std::vector<Node> MotionPlanner::SelectMotion(std::vector<std::vector<Node>> motionPrimitives)
{
  /*
    TODO: select the minimum cost motion primitive
  
    1. Calculate cost terms
    2. Calculate total cost (weighted sum of all cost terms)
    3. Compare & Find minimum cost (double minCost) & minimum cost motion (std::vector<Node> motionMinCost)
    4. Return minimum cost motion
  */
  const double W_GOAL = 1.0;
  const double W_HEAD = 0.5;
  const double W_STEER = 0.2;
  const double W_TRAV = this->W_COST_TRAVERSABILITY; 
  const double W_PROG = 0.3;

  double minCost = 9999999;
  std::vector<Node> motionMinCost; // initialize as odom

  // check size of motion primitives
  if (motionPrimitives.size() != 0) {
    // Iterate all motion primitive (motionPrimitive) in motionPrimitives
    for (auto& motionPrimitive : motionPrimitives) {
      //!1. Calculate cost terms
      // <<MODIFICATION>>
      if (motionPrimitive.empty()) continue;
      double J_goal = motionPrimitive.back().minDistGoal;
      double J_steer = std::abs(motionPrimitive.back().delta);
      double J_trav = motionPrimitive.back().cost_colli;
      double J_head = 0.0;
      double progress_m = std::hypot(motionPrimitive.back().x - motionPrimitive.front().x, motionPrimitive.back().y - motionPrimitive.front().y);
      // double progress_m = std::hypot(motionPrimitive.back().x, motionPrimitive.back().y);
      if (this->bGetLocalNode){
        double th_goal = std::atan2(this->truncLocalNode.y - motionPrimitive.back().y, 
                                    this->truncLocalNode.x - motionPrimitive.back().x);
        J_head = std::abs(normalizePiToPi(th_goal - motionPrimitive.back().yaw));
      }

      //! 2. Calculate total cost ex) collision cost, goal distance, goal direction, progress cost, steering cost....
      // <<MODIFICATION>>
      double cost_total =
        W_GOAL * J_goal +
        W_HEAD * J_head +
        W_STEER * J_steer +
        W_TRAV * J_trav -
        W_PROG * progress_m;
        
      // ROS_INFO_STREAM("COST TOTAL: " << cost_total);
      motionPrimitive.back().cost_total = cost_total;

      ROS_INFO_STREAM_THROTTLE(1.0, "[Cost] J_goal=" << J_goal
      << " J_head=" << J_head << " J_steer=" << J_steer
      << " J_trav=" << J_trav << " prog=" << progress_m
      << " => total=" << cost_total
      << " len=" << motionPrimitive.size()
      << " collided=" << (motionPrimitive.back().collision ? "Y":"N"));

      //! 3. Compare & Find minimum cost & minimum cost motion
      if (cost_total < minCost) {
          motionMinCost = motionPrimitive;
          minCost = cost_total;
      }
    }
  }
  //<<MODIFICATION>>
  if (!motionMinCost.empty()) {
    const auto& last = motionMinCost.back();
    ROS_INFO_STREAM_THROTTLE(1.0, "[Pick] v_est=" 
      << (0.1 * this->MOTION_VEL * motionMinCost.size() /
          (this->MAX_PROGRESS / this->DIST_RESOL))
      << " delta=" << last.delta
      << " yaw=" << last.yaw
      << " cost=" << last.cost_total
      << " len=" << motionMinCost.size());
  } 
  else {
    ROS_ERROR_THROTTLE(1.0, "[Pick] No valid primitive was found!");
  }
  //! 4. Return minimum cost motion
  return motionMinCost;
}

/* ----- Util Functions ----- */

bool MotionPlanner::CheckCollision(Node goalNodePlanner, nav_msgs::OccupancyGrid localMap)
{
  /*
    TODO: check collision of the current node
    - the position x of the node should be in a range of [0, map width]
    - the position y of the node should be in a range of [0, map height]
    - check all map values within the inflation area of the current node
    e.g.,

    for loop i in range(0, inflation_size)
      for loop j in range(0, inflation_size)
        tmp_x := currentNodeMap.x + i - 0.5*inflation_size <- you need to check whether this tmp_x is in [0, map width]
        tmp_y := currentNodeMap.y + j - 0.5*inflation_size <- you need to check whether this tmp_x is in [0, map height]
        map_index := "index of the grid at the position (tmp_x, tmp_y)" <-- map_index should be int, not double!
        map_value = static_cast<int16_t>(localMap.data[map_index])
        if (map_value > map_value_threshold) OR (map_value < 0)
          return true
    return false

    - use params in header file: INFLATION_SIZE, OCCUPANCY_THRES
  */
  Node currentNodeMap = goalNodePlanner;
  const int width  = static_cast<int>(localMap.info.width);
  const int height = static_cast<int>(localMap.info.height);
  const int inflation_size =  static_cast<int>(std::round(this->INFLATION_SIZE));
  

  for (int i = 0; i < inflation_size; ++i) {
    for (int j = 0; j < inflation_size; ++j) {
      double tmp_x = currentNodeMap.x + static_cast<double>(i) - 0.5 * static_cast<double>(inflation_size);
      double tmp_y = currentNodeMap.y + static_cast<double>(j) - 0.5 * static_cast<double>(inflation_size);
      int gx = static_cast<int>(std::round(tmp_x));
      int gy = static_cast<int>(std::round(tmp_y));
      // check whether this tmp_x/tmp_y is in [0, width/height]
      if (gx < 0 || gx >= width || gy < 0 || gy >= height) {
        ROS_WARN_STREAM_THROTTLE(1.0, "[Collision] OOB gx=" << gx << " gy=" << gy
        << " (w,h)=" << width << "," << height);
        return true;  // treat out-of-bounds as collision
      }
      int map_index = width * gy + gx;
      int16_t map_value = static_cast<int16_t>(localMap.data[map_index]);
      if (map_value > this->OCCUPANCY_THRES || map_value < 0) {
        ROS_WARN_STREAM_THROTTLE(1.0, "[Collision] cell=" << (width*gy+gx)
        << " val=" << (int)localMap.data[map_index]);
        return true;
      }
      // Only values > threshold are collision
      if (map_value > this->OCCUPANCY_THRES || map_value < 0) {
        // Optional debug:
        ROS_WARN_STREAM_THROTTLE(1.0, "[Collision] cell=" << map_index
                                    << " val=" << static_cast<int>(map_value)
                                    << " gx=" << gx << " gy=" << gy);
        return true;
      }
    }
  }
  return false;
}

bool MotionPlanner::CheckRunCondition()
{
  if (this->bGetMap && this->bGetGoal) {
  // if (this->bGetMap) {
    return true;
  }
  else {
    std::cout << "Run condition is not satisfied!!!" << "bGetMap : " << bGetMap << " bGetGoal : " << bGetGoal << std::endl;
    return false;
  }
}

Node MotionPlanner::GlobalToLocalCoordinate(Node globalNode, nav_msgs::Odometry egoOdom)
{
  // Coordinate transformation from global to local
  Node tmpLocalNode;
  // - Copy data globalNode to tmpLocalNode
  memcpy(&tmpLocalNode, &globalNode, sizeof(struct Node));

  // - Coordinate transform
  // -- translatioonal transform
  double delX = globalNode.x - egoOdom.pose.pose.position.x;
  double delY = globalNode.y - egoOdom.pose.pose.position.y;
  double delZ = globalNode.z - egoOdom.pose.pose.position.z;

  // -- rotational transform
  tf2::Quaternion q_ego;
  double egoR, egoP, egoY;
  // --- copy quaternion from odom
  tf2::convert(egoOdom.pose.pose.orientation, q_ego);
  // --- get roll pitch yaw
  tf2::Matrix3x3 m_odom(q_ego);
  m_odom.getRPY(egoR, egoP, egoY);

  // - calculate new pose
  double newX = cos(-egoY) * delX - sin(-egoY) * delY;
  double newY = sin(-egoY) * delX + cos(-egoY) * delY;
  double newZ = delZ;
  double newYaw = globalNode.yaw - egoY;

  // - Update pose
  tmpLocalNode.x = newX;
  tmpLocalNode.y = newY;
  tmpLocalNode.z = newZ;
  tmpLocalNode.yaw = newYaw;

  return tmpLocalNode;
}

geometry_msgs::PoseStamped MotionPlanner::GlobalToLocalCoordinate(geometry_msgs::PoseStamped poseGlobal, nav_msgs::Odometry egoOdom)
{
  // Coordinate transformation from global to local
  // - Copy data nodeGlobal to nodeLocal
  geometry_msgs::PoseStamped poseLocal;

  // - Coordinate transform
  // -- translatioonal transform
  double delX = poseGlobal.pose.position.x - egoOdom.pose.pose.position.x;
  double delY = poseGlobal.pose.position.y - egoOdom.pose.pose.position.y;
  double delZ = poseGlobal.pose.position.z - egoOdom.pose.pose.position.z;

  // -- rotational transform
  tf2::Quaternion q_goal, q_ego;
  double goalR, goalP, goalY;
  double egoR, egoP, egoY;
  // --- copy quaternion from odom
  tf2::convert(poseGlobal.pose.orientation, q_goal);
  tf2::convert(egoOdom.pose.pose.orientation, q_ego);
  // --- get roll pitch yaw
  tf2::Matrix3x3 m_goal(q_goal);
  tf2::Matrix3x3 m_odom(q_ego);
  m_goal.getRPY(goalR, goalP, goalY);
  m_odom.getRPY(egoR, egoP, egoY);

  // - calculate new pose
  double newX = cos(-egoY) * delX - sin(-egoY) * delY;
  double newY = sin(-egoY) * delX + cos(-egoY) * delY;
  double newZ = delZ;
  double newYaw = goalY - egoY;

  // - Update pose
  // -- quaternion to RPY 
  // -- RPY to Quaternion
  tf2::Quaternion globalQ, globalQ_new;
  globalQ.setRPY(0.0, 0.0, newYaw);
  globalQ_new = globalQ.normalize();

  poseLocal.pose.position.x = newX;
  poseLocal.pose.position.y = newY;
  poseLocal.pose.position.z = newZ;
  tf2::convert(globalQ_new, poseLocal.pose.orientation);

  return poseLocal;
}

Node MotionPlanner::LocalToPlannerCorrdinate(Node nodeLocal)
{
    /*
      TODO: Transform from local to occupancy grid map coordinate
      - local coordinate ([m]): x [map min x, map max x], y [map min y, map max y]
      - map coordinate ([cell]): x [0, map width], y [map height]
      - convert [m] to [cell] using map resolution ([m]/[cell])
    */

    Node nodeMap;
    memcpy(&nodeMap, &nodeLocal, sizeof(struct Node));

    try {
        // Use classic tf instead of tf2 for ROS Noetic
        tf::StampedTransform transform;
        tf_listener_.lookupTransform("camera_link", "base_link", ros::Time(0), transform);
        
        // Transform the point from base_link to camera_link
        tf::Point point_base(nodeLocal.x, nodeLocal.y, 0.0);
        tf::Point point_camera = transform * point_base;
        
        // Apply the slide's formula in the correct frame (camera_link)
        double grid_x = (point_camera.x() - this->origin_x) / this->mapResol;
        double grid_y = (point_camera.y() - this->origin_y) / this->mapResol;
        
        nodeMap.x = static_cast<int>(std::round(grid_x));
        nodeMap.y = static_cast<int>(std::round(grid_y));
        
        ROS_WARN_STREAM_THROTTLE(1.0, "[COORD_FIXED] Base(" << nodeLocal.x << "," << nodeLocal.y 
                                << ") → Camera(" << point_camera.x() << "," << point_camera.y()
                                << ") → Grid(" << nodeMap.x << "," << nodeMap.y << ")");
        
    } catch (tf::TransformException &ex) {
        ROS_ERROR_STREAM("TF transformation failed: " << ex.what());
        // Emergency fallback
        double grid_x = (nodeLocal.x - this->origin_x) / this->mapResol;
        double grid_y = (nodeLocal.y - this->origin_y) / this->mapResol;
        nodeMap.x = static_cast<int>(std::round(grid_x));
        nodeMap.y = static_cast<int>(std::round(grid_y));
    }
    
    return nodeMap;
}

/* ----- Publisher ----- */

void MotionPlanner::PublishData(std::vector<Node> motionMinCost, std::vector<std::vector<Node>> motionPrimitives)
{
  // Publisher
  // - visualize selected motion primitive
  PublishSelectedMotion(motionMinCost);
  // - visualize motion primitives
  PublishMotionPrimitives(motionPrimitives);
  // - publish command
  PublishCommand(motionMinCost);
}

/* ----- Main ----- */

int main(int argc, char* argv[])
{ 
  std::cout << "start main process" << std::endl;

  ros::init(argc, argv, "control_space_planner");
  // for subscribe
  ros::NodeHandle nh;
  ros::Rate rate(50.0);
  MotionPlanner MotionPlanner(nh);

  // Planning loop
  while (MotionPlanner.nh_.ok()) {
      // Spin ROS
      ros::spinOnce();
      // check run condition
      if (MotionPlanner.CheckRunCondition()) {
        // Run algorithm
        MotionPlanner.Plan();
      }
      rate.sleep();
  }

  return 0;
}
