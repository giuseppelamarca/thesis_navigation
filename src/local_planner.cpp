#include "local_planner.h"
#include <iostream>
#include <fstream>
#include <pluginlib/class_list_macros.h>
#include <base_local_planner/goal_functions.h>


 //register this planner as a BaseGlobalPlanner plugin
 PLUGINLIB_EXPORT_CLASS(local_planner::LocalPlanner, nav_core::BaseLocalPlanner)

 using namespace std;
  ros::NodeHandle private_nh;

  class Point{
  public:
    int x, y;
    Point(int,int);
    bool operator==(const Point &b){
      if (this->x == b.x && this->y == b.y)
        return true;
      else
        return false;
    }
};
Point::Point(int a, int b){
  x = a;
  y = b;
}


void local_costmap_callback(const nav_msgs::OccupancyGrid &costmap){
  ROS_INFO("COSTMAP VALUE: %d",costmap.data[0]);
}

 //Default Constructor
 namespace local_planner {
int LocalPlanner::nearestPoint(const int start_point, const tf::Stamped<tf::Pose> & pose) {
    int plan_point = start_point;
    double best_metric = std::numeric_limits<double>::max();
    //// ACTIVATING THIS PIECE OF CODE FORCE THE ROBOT TO PASS AS CLOSE AS POSSIBLE TO THE GLOBAL PLAN ///
    double start_dist = base_local_planner::getGoalPositionDistance(pose, plan_[start_point].pose.position.x, plan_[start_point].pose.position.y);
    if (start_dist<0.7)
      visited_index.push_back(start_point);
    //// END ACTIVATION ///
    for( int i=start_point; i<plan_.size(); i++ ) {
      double dist = base_local_planner::getGoalPositionDistance(pose, plan_[i].pose.position.x, plan_[i].pose.position.y);
      double metric = dist;
      if( metric < best_metric && std::find(visited_index.begin(), visited_index.end(), i)==visited_index.end()) {
        best_metric = metric;
        plan_point = i;
        for (int j = start_point; j < plan_point; j++)
          visited_index.push_back(j);
      }
    }
    for (int i =0; i<visited_index.size(); i++)
    return plan_point;
}

bool LocalPlanner::safety_path(const int next_point, const float yaw){
    float dist = 1.5; 
    float width = 0.5; 
    float safety_angle = atan2(width,dist);
}

void LocalPlanner::index_calculation_costmap(unsigned int *c_x,unsigned int *c_y, float yaw){
  //// INDEX CALCULATION FOR LOCAL COSTMAP
  float yaw_temp = yaw / 0.017453 ;        //this allow to do all the calculations refering to deg
  *c_x = 0; 
  *c_y = 0;
  float res = 3.33; 
  int offset = 150; 

  if (abs(yaw_temp)<45){
    *c_x = 299; 
    *c_y = yaw_temp * res + offset; 
  }
  else if(abs(yaw_temp)>45 && abs(yaw_temp)<135){
    if(yaw_temp>0){
      *c_x = -(yaw_temp - 90) * res + offset;
      *c_y = 299; 
    }
    else{
      *c_x = -(yaw_temp - 90) * res + offset;
      *c_y = 0; 
    }
  }
  else if(abs(yaw_temp)>135){
    if (yaw_temp>0){
        *c_x = 0; 
        *c_y = -(yaw_temp - 180) * res + offset;
    }
    else{
        *c_x = 0; 
        *c_y = -(yaw_temp + 180) * res + offset;
    }
  }
  //// END INDEX CALCULATION FOR LOCAL COSTMAP
}

 LocalPlanner::LocalPlanner (){
 }

 LocalPlanner::LocalPlanner(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS *costmap_ros){
   initialize(name, tf, costmap_ros);
 }

 void LocalPlanner::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS *costmap_ros){
  costmap_ros_ = costmap_ros; 
  ROS_INFO("LOCAL COSTMAP BRO");
  ROS_INFO("local costmap size: %d, %d",costmap_ros_->getCostmap()->getSizeInCellsX(), costmap_ros_->getCostmap()->getSizeInCellsY());
  ROS_INFO("local costmap resolution: %f",costmap_ros_->getCostmap()->getResolution());
  private_nh = ros::NodeHandle("~/" + name);
  l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
  g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
  private_nh.setParam("/k1", 0.3);
  private_nh.setParam("/k1", 0.1);
  private_nh.setParam("/u1_v", 0.2);
  private_nh.setParam("/u2_v", 0.2);

  ros::NodeHandle nh;
  ros::Subscriber local_costmap = nh.subscribe("/move_base/local_costmap/costmap", 1, local_costmap_callback);
 }

 bool LocalPlanner::computeVelocityCommands (geometry_msgs::Twist &cmd_vel){
  tf::Stamped<tf::Pose> current_pose;
  costmap_ros_->getRobotPose(current_pose);
  index_to_plan++;


  int plan_point = nearestPoint(last_plan_point_, current_pose);

  last_plan_point_ = plan_point;

  if (plan_point < plan_.size()){
    int i  = plan_point + 1; 
    geometry_msgs::PoseStamped next_pose = plan_[i];

    std::vector<geometry_msgs::PoseStamped> local_plan;
    for(int i = 0; i< 5; i++){
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = costmap_ros_->getGlobalFrameID();
      pose.pose.position.x = plan_[i].pose.position.x;
      pose.pose.position.y = plan_[i].pose.position.y;
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
      local_plan.push_back(pose);
    }
    base_local_planner::publishPlan(local_plan, l_plan_pub_);  
    float linear_vel = 0.0;
    float angular_vel = 0.0;

    tf::Quaternion q(
        current_pose.getRotation().x(),
        current_pose.getRotation().y(),
        current_pose.getRotation().z(),
        current_pose.getRotation().w());
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // INITIAL ROTATION TO NEXT POSE ALLIGNMENT
    float next_point_angle = atan2(next_pose.pose.position.y -  current_pose.getOrigin().y(),next_pose.pose.position.x -  current_pose.getOrigin().x());

    if (abs(yaw-next_point_angle)>1){
      initial_allignment = true;
      ROS_INFO("allignment.....");
    }
    
    /// INPUT OUTPUT LINEARIZATION
    float MIN_LIN_VEL = 0.01;
    float MAX_LIN_VEL = 0.2;
    float MIN_ANG_VEL = -0.2;
    float MAX_ANG_VEL = 0.2;
    float b =  0.35;
    float k1 = 0.3; 
    float k2 = 0.15;
    float u1_v = 0;
    float u2_v = 0;
    private_nh.getParam("/k1", k1);
    private_nh.getParam("/k2", k2);
    private_nh.getParam("/u1_v", u1_v);
    private_nh.getParam("/u2_v", u2_v);
    float u1 =  u1_v + k1 * (next_pose.pose.position.x -  current_pose.getOrigin().x());
    float u2 =  u2_v+ k2 * (next_pose.pose.position.y -  current_pose.getOrigin().y());

    /// END  INPUT OUTPUT LINEARIZATION

    /// CHECK IF POSITION REACHED AND ROTATE
    float robot_pos_x = current_pose.getOrigin().x();
    float robot_pos_y = current_pose.getOrigin().y();
    if (sqrt(pow(robot_pos_x - goal_pos_x,2)+pow(robot_pos_y - goal_pos_y,2))< 0.2){
      ROS_INFO("position reached");
      position_reached = true; 
      linear_vel = 0;
      angular_vel = 0;
    }
    //The robot is in the goal position and need just to rotate to allineate with the goal orientation
    if (position_reached || initial_allignment){
      if(position_reached){
        tf::Quaternion quat_goal(
          plan_[plan_.size()-1].pose.orientation.x,
          plan_[plan_.size()-1].pose.orientation.y,
          plan_[plan_.size()-1].pose.orientation.z,
          plan_[plan_.size()-1].pose.orientation.w);
      }
      else{
        tf::Quaternion quat_goal(
          plan_[i].pose.orientation.x,
          plan_[i].pose.orientation.y,
          plan_[i].pose.orientation.z,
          plan_[i].pose.orientation.w);
      }
      tf::Matrix3x3 m_goal(quat_goal);
      double roll_goal, pitch_goal, yaw_goal;
      m_goal.getRPY(roll_goal, pitch_goal, yaw_goal);
      linear_vel = 0;
      double P = 0.5;
      angular_vel = P * (yaw_goal - yaw);
      if (abs(angular_vel) < 0.05){
        ROS_INFO("angular velocity: %f", angular_vel);
        position_reached = false; 
        initial_allignment = false;
        goal_reached = true;
      }
    }
    else{
    linear_vel = cos(yaw) * u1 + sin(yaw) * u2;
    angular_vel = - sin(yaw) / b * u1 + cos(yaw) / b * u2;

    
    if (linear_vel > 0)
      linear_vel = min(linear_vel, MAX_LIN_VEL);
    else
      linear_vel = max(linear_vel, MIN_LIN_VEL);

    if (angular_vel > 0)
      angular_vel = min(angular_vel, MAX_ANG_VEL);
    else
      angular_vel = max(angular_vel, MIN_ANG_VEL);
    }
    
    //// END CHECK POSITION REACHED
    ROS_INFO("linear: %f \t angular: %f", linear_vel, angular_vel);



    cmd_vel.linear.x = linear_vel; 
    cmd_vel.linear.y = linear_vel; 
    cmd_vel.linear.z = 0; 

    cmd_vel.angular.z = angular_vel; 
  }
  return valid_velocity;
 }

 bool LocalPlanner::isGoalReached (){
  index_to_plan = 0;
  if (goal_reached)
    ROS_INFO("goal reached");
  return goal_reached; 
 }

 bool LocalPlanner::setPlan (const std::vector<geometry_msgs::PoseStamped>& plan){
  goal_reached = false;
  plan_ = plan;
  last_plan_point_ = 0;
  visited_index.clear();
  start_rotation = false;
  old_delta = 0;
  old_linear_vel = 0.01;

    cell_x_size = costmap_ros_->getCostmap()->getSizeInCellsX();
    cell_y_size = costmap_ros_->getCostmap()->getSizeInCellsY();

  goal_pos_x = plan_[plan_.size()-1].pose.position.x;
  goal_pos_y = plan_[plan_.size()-1].pose.position.y;
  ROS_INFO("LOCAL PLAN SIZE: %d", plan.size());

  for (int i = 0; i < plan.size(); i++)
    ROS_INFO("x: %f \t y: %f", plan_[i].pose.position.x, plan_[i].pose.position.y);

  base_local_planner::publishPlan(plan, g_plan_pub_);

  for (int i = 0; i<300;i++)
  for(int j=0; j<300; j++){
    local_costmap_matrixI(i,j) = i;
    local_costmap_matrixJ(i,j) = j; 
    rotation_matrix(i,j) = 0; 

  }

  return true; 
  }
 };
