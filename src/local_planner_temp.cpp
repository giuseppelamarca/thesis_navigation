#include "local_planner.h"
#include <iostream>
#include <fstream>
#include <pluginlib/class_list_macros.h>
#include <base_local_planner/goal_functions.h>

 //register this planner as a BaseGlobalPlanner plugin
 PLUGINLIB_EXPORT_CLASS(local_planner::LocalPlanner, nav_core::BaseLocalPlanner)

 using namespace std;

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

/*
 int LocalPlanner::nearestPoint(const int start_point, const tf::Stamped<tf::Pose> & pose) {
    int plan_point = start_point;
    double best_metric = std::numeric_limits<double>::max();
    //// ACTIVATING THIS PIECE OF CODE FORCE THE ROBOT TO PASS AS CLOSE AS POSSIBLE TO THE GLOBAL PLAN ///
    double start_dist = base_local_planner::getGoalPositionDistance(pose, plan_[start_point].pose.position.x, plan_[start_point].pose.position.y);
    if (start_dist>0.3)
      return start_point;
    //// END ACTIVATION ///
    for( int i=start_point; i<plan_.size(); i++ ) {
      double dist = base_local_planner::getGoalPositionDistance(pose, plan_[i].pose.position.x, plan_[i].pose.position.y);
      //ROS_INFO("pose in nearest_point: %f, %f", pose.getOrigin().x(), pose.getOrigin().y());
      //ROS_INFO("dist: %f", dist);
      double metric = dist;
      //ROS_INFO_NAMED("ackermann_planner", "Distance to path: %f", dist);
      if( metric < best_metric && std::find(visited_index.begin(), visited_index.end(), i)==visited_index.end()) {
        visited_index.push_back(i);
        best_metric = metric;
        plan_point = i;
      }
    }
    for (int i =0; i<visited_index.size(); i++)
      //ROS_INFO("visited_index: %d", visited_index[i]);
    return plan_point;
  }*/

int LocalPlanner::nearestPoint(const int start_point, const tf::Stamped<tf::Pose> & pose) {
    int plan_point = start_point;
    double best_metric = std::numeric_limits<double>::max();
    //// ACTIVATING THIS PIECE OF CODE FORCE THE ROBOT TO PASS AS CLOSE AS POSSIBLE TO THE GLOBAL PLAN ///
    //// if one of the point in the middle of the path is close to the position of the robot all the point from previous position to actual position are considere visited
    if (base_local_planner::getGoalPositionDistance(pose, plan_[plan_.size()-1].pose.position.x, plan_[plan_.size()-1].pose.position.y) < 0.2){
        for(int j = start_point; j< plan_.size()-1; j++){
            if (std::find(visited_index.begin(), visited_index.end(), j)==visited_index.end())
                visited_index.push_back(j);
        }
    }
    else{
        //do the check leaving the last point before reaching the end of the path
        for (int i=start_point; i<plan_.size();i++){
            double start_dist = base_local_planner::getGoalPositionDistance(pose, plan_[i].pose.position.x, plan_[i].pose.position.y);
            //ROS_INFO("INDEX: %d     DIST: %f", i, start_dist);
            if (start_dist<0.7){   //0.7
                for(int j = start_point; j<= i; j++){
                    if (std::find(visited_index.begin(), visited_index.end(), j)==visited_index.end())
                        visited_index.push_back(j);
                }
            }
        }
    }
    //// END ACTIVATION ///
    for( int i=start_point; i<plan_.size(); i++ ) {
      double dist = base_local_planner::getGoalPositionDistance(pose, plan_[i].pose.position.x, plan_[i].pose.position.y);
      //ROS_INFO("pose in nearest_point: %f, %f", pose.getOrigin().x(), pose.getOrigin().y());
      //ROS_INFO("dist: %f", dist);
      double metric = dist;
      //ROS_INFO_NAMED("ackermann_planner", "Distance to path: %f", dist);
      if( metric < best_metric && std::find(visited_index.begin(), visited_index.end(), i)==visited_index.end()) {
      ROS_INFO("metric is smaller");
        best_metric = metric;
        plan_point = i;
        for (int j = start_point; j < plan_point; j++)
            if (std::find(visited_index.begin(), visited_index.end(), j)==visited_index.end())
              visited_index.push_back(j);
      }
    }
    //ROS_INFO("target index: %d", plan_.size()-1);
    for (int i =0; i<visited_index.size(); i++){
      //ROS_INFO("visited_index: %d", visited_index[i]);
    }
    //ROS_INFO("RETURNED PLAN POINT: %d", plan_point);
    return plan_point;
}

bool LocalPlanner::safety_path(const int next_point, const float yaw){
    float dist = 1.5; 
    float width = 0.5; 
    float safety_angle = atan2(width,dist);
    //yaw =  yaw + safety_angle;
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
  ros::NodeHandle private_nh("~/" + name);
  l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
  g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
  ros::NodeHandle nh;
  ros::Subscriber local_costmap = nh.subscribe("/move_base/local_costmap/costmap", 1, local_costmap_callback);
 }

 bool LocalPlanner::computeVelocityCommands (geometry_msgs::Twist &cmd_vel){
  tf::Stamped<tf::Pose> current_pose;
  costmap_ros_->getRobotPose(current_pose);
  index_to_plan++;

  int plan_point = nearestPoint(last_plan_point_, current_pose);
  //  ROS_INFO("pos robot: %f, pos_plan: %f, last pos_plan_next: %f", current_pose.getOrigin().x(), plan_[plan_point].pose.position.x,plan_[plan_.size()-1].pose.position.x);
  // ROS_INFO("plan point: %d, plan size: %d", plan_point, plan_.size());
  last_plan_point_ = plan_point;

/*
  geometry_msgs::PoseStamped extended_plan = plan_[plan_.size()-1];
  while(plan_.size()<6){
    plan_.push_back(extended_plan);
  }
*/
  
  if (plan_point < plan_.size()-1){
    int i  = plan_point + 1; 
    geometry_msgs::PoseStamped next_pose = plan_[i];

    std::vector<geometry_msgs::PoseStamped> local_plan;
    //ROS_INFO("before push back local planner");
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
    float old_angular_vel = 0.0;


    tf::Quaternion q(
        current_pose.getRotation().x(),
        current_pose.getRotation().y(),
        current_pose.getRotation().z(),
        current_pose.getRotation().w());
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    //ROS_INFO("robot orientation: %f", yaw);
   //ROS_INFO("after orientation");
    
    
    
    
    /// INPUT OUTPUT LINEARIZATION
    float MIN_LIN_VEL = -0.0;
    float MAX_LIN_VEL = 0.3;
    float MIN_ANG_VEL = -0.2;
    float MAX_ANG_VEL = 0.2;
    float b =  0.2;//0.35;
    float k1 = 2.5;
    float k2 = 1;
    float u1 = 0.1 + k1 * (next_pose.pose.position.x -  current_pose.getOrigin().x());
    float u2 = 0.1 + k2 * (next_pose.pose.position.y -  current_pose.getOrigin().y());
    //ROS_INFO("after u choose");


    ////////activate else branch of the CHECK IF POSITION REACHED AND ROTATE
    /// END  INPUT OUTPUT LINEARIZATION




    //// VFF METHOD  
/*
    float Fr = 0.03;
    float Ft = 0.15; 
    float FR[2] = {0,0};
    float FR_damp[2] = {0,0};    
    float FT[2] = {0,0};
    float R[2] = {0,0};
    float T = 0.1;
    float tau = 0.3;
    int robot_cell = cell_x_size/2;
    for (int i = 0; i < cell_x_size; i++){
      for (int j = 0; j < cell_y_size; j++){
        if (i == robot_cell && j == robot_cell)
          continue;
        float d = sqrt((robot_cell - i)*(robot_cell - i) + (robot_cell - j)*(robot_cell - j) ); 
        float temp = (Fr * costmap_ros_->getCostmap()->getCost(i,j) ) / (d * d); 
        FR[0] += temp * ( i - robot_cell) * 0.01 /d ;
        FR[1] += temp * ( j - robot_cell) * 0.01 /d ;
      }
    }
    float w = 0.25;
    FR[0] = -FR[0];
    FR[1] = -FR[1];
    float cos_teta = (old_linear_vel*cos(yaw) * FR[0] + old_linear_vel*sin(yaw) * FR[1])/(old_linear_vel*(sqrt(FR[0]*FR[0]+FR[1]*FR[1])+0.001));
    FR_damp[0] = w * FR[0] + (1 - w) * FR[0] * (-cos_teta);
    FR_damp[1] = w * FR[1] + (1 - w) * FR[1] * (-cos_teta);
    float x_dist = next_pose.pose.position.x -  current_pose.getOrigin().x();
    float y_dist = next_pose.pose.position.y -  current_pose.getOrigin().y();
    float target_dist = sqrt(x_dist * x_dist + y_dist * y_dist);
    FT[0] =  Ft * x_dist / target_dist; 
    FT[1] =  Ft * y_dist / target_dist;
    R[0] = FR_damp[0] + FT[0];
    R[1] = FR_damp[1] + FT[1];
    float delta = atan2(R[1],R[0]);
    float new_delta = (T * delta + (tau - T) * old_delta)/tau;
    old_delta = delta;
    angular_vel = 1.5 * (new_delta - yaw);
    //ROS_INFO("cos teta: %f", cos_teta);

    linear_vel = 0.5*(1-abs(cos_teta));
        ROS_INFO("angular vel: %f linear vel: %f", angular_vel, linear_vel);

    old_linear_vel = linear_vel;




    //// END VFF 
*/
    ///// VFH METHOD

    /*
    if (plan_.size()<5){
      position_reached = true;
    }
    */
    int costmap_size = costmap_ros_->getCostmap()->getSizeInCellsX();

    
    //float B1[costmap_size/2][costmap_size];
    //float m1[costmap_size/2][costmap_size];
    
    vector<float> h; 
    vector<int> h_index;
    vector<int> h_free;
    float alpha = 5; 
    int n = 72;
    float b1 = 0.001;
    float a1 = b1*sqrt(2)*1.5;
    
    int sector = 0; 
    int sector_robot = 0; 
    int threshold = 10000; 

    bool front = true; 

    float target_angle_temp = atan2(next_pose.pose.position.y -  current_pose.getOrigin().y(),next_pose.pose.position.x -  current_pose.getOrigin().x());
    
    if (target_angle_temp<= 0.7853981634   && target_angle_temp >= -0.7853981634)
      sector = 1; 
    else if (target_angle_temp<= 2.3561944902   && target_angle_temp > 0.7853981634)
      sector = 2;
    else if (target_angle_temp< -0.7853981634   && target_angle_temp >=-2.3561944902 )
      sector = 3;
    else
      sector = 4;

    if ( yaw <= 0.7853981634   && yaw >= -0.7853981634)
      sector_robot = 1; 
    else if (yaw <= 2.3561944902   && yaw > 0.7853981634)
      sector_robot  = 2;
    else if (yaw < -0.7853981634   && yaw >=-2.3561944902 )
      sector_robot  = 3;
    else
      sector_robot  = 4;
    
/*
    if (yaw<= 0.7853981634 && yaw>= -0.7853981634 ){
    ROS_INFO("SECTOR1");
    //if (sector ==1){
      float B1[costmap_size/2][costmap_size];
      float m1[costmap_size/2][costmap_size];
      //i  da 150 a 299 
      //j da 0 a 299
      for (int l = 0; l< n; l++){
        h_index.push_back(l*alpha);
        h.push_back(0);
        
        for (int i = costmap_size/2; i< costmap_size;i++){
            for(int j = 0; j< costmap_size; j++){
              int cell_cost = costmap_ros_->getCostmap()->getCost(i,j);
              float x_dist = (i-costmap_size/2)*costmap_ros_->getCostmap()->getResolution();
              float y_dist = (j-costmap_size/2)*costmap_ros_->getCostmap()->getResolution();
              B1[i-costmap_size/2][j]=atan2(y_dist, x_dist);
              //ROS_INFO("B: %f", B1[i-150][j] );
              m1[i-costmap_size/2][j]=cell_cost*cell_cost * (a1-b1*sqrt(x_dist*x_dist+y_dist*y_dist));
              if(alpha*int((B1[i-costmap_size/2][j]+1.5707963268)/alpha/0.0174532925)==h_index[l])
                h[l]+=m1[i-costmap_size/2][j];
            }
        }
	if (h_index[l] <200){
	//	ROS_INFO("h %d: %f", h_index[l],h[l]);
	}
        
      }   
    }
*/
    if ( sector == 1 ){
    ROS_INFO("SECTOR1");
    //if (sector ==1){
      float B1[costmap_size/2][costmap_size];
      float m1[costmap_size/2][costmap_size];
      //i  da 150 a 299 
      //j da 0 a 299
      for (int l = 0; l< n; l++){
        h_index.push_back(l*alpha);
        h.push_back(0);
        
        for (int i = costmap_size/2; i< costmap_size;i++){
            for(int j = 0; j< costmap_size; j++){
              int cell_cost = costmap_ros_->getCostmap()->getCost(i,j);
              float x_dist = (i-costmap_size/2)*costmap_ros_->getCostmap()->getResolution();
              float y_dist = (j-costmap_size/2)*costmap_ros_->getCostmap()->getResolution();
              B1[i-costmap_size/2][j]=atan2(y_dist, x_dist);
              //ROS_INFO("B: %f", B1[i-150][j] );
              m1[i-costmap_size/2][j]=cell_cost*cell_cost * (a1-b1*sqrt(x_dist*x_dist+y_dist*y_dist));
              if(alpha*int((B1[i-costmap_size/2][j]+1.5707963268)/alpha/0.0174532925)==h_index[l])
                h[l]+=m1[i-costmap_size/2][j];
            }
        }
	//ROS_INFO("h %d: %f", h_index[l],h[l]);
	
        
      }   
    }

    else if ( sector == 2 ){
    ROS_INFO("SECTOR2");
    //else if (sector == 2){
      float B1[costmap_size][costmap_size/2];
      float m1[costmap_size][costmap_size/2];

      //i da 0 a 299
      //j da 150 a 299
      front = false;
      for (int l = 0; l< n; l++){
        h_index.push_back(l*alpha);
        h.push_back(0);
        for (int i = 0; i< costmap_size;i++){
            for(int j = costmap_size/2; j< costmap_size; j++){
              int cell_cost = costmap_ros_->getCostmap()->getCost(i,j);
              float x_dist = (i-costmap_size/2)*costmap_ros_->getCostmap()->getResolution();
              float y_dist = (j-costmap_size/2)*costmap_ros_->getCostmap()->getResolution();
              B1[i][j - costmap_size/2]=atan2(y_dist, x_dist);
              m1[i][j - costmap_size/2]=cell_cost*cell_cost * (a1-b1*sqrt(x_dist*x_dist+y_dist*y_dist));
              if(alpha*int((B1[i][j - costmap_size/2]+1.5707963268)/alpha/0.0174532925)==h_index[l])
                h[l]+=m1[i][j - costmap_size/2];
            }
        }
        //ROS_INFO("h %d: %f", h_index[l],h[l]);
      }
    }
    else if ( sector == 3 ){
    ROS_INFO("SECTOR3");
    //else if (sector == 3){
      float B1[costmap_size][costmap_size/2];
      float m1[costmap_size][costmap_size/2];

      //i da 0 a 299
      //j da 0 a 150
      ROS_INFO("SONO NELL'ELSE");
      front = false;
      for (int l = 0; l< n; l++){
        h_index.push_back(l*alpha);
        h.push_back(0);
        for (int i = 0; i< costmap_size;i++){
            for(int j = costmap_size/2; j>0; j--){
              int cell_cost = costmap_ros_->getCostmap()->getCost(i,j);
              //float x_dist = (costmap_size/2 - i)*costmap_ros_->getCostmap()->getResolution();
              //float y_dist = (j - costmap_size/2)*costmap_ros_->getCostmap()->getResolution();
              float x_dist = (i - costmap_size/2)*costmap_ros_->getCostmap()->getResolution();
              float y_dist = (j - costmap_size/2)*costmap_ros_->getCostmap()->getResolution();
              B1[i][costmap_size/2 - j]=atan2(y_dist, x_dist);
              m1[i][costmap_size/2 - j]=cell_cost*cell_cost * (a1-b1*sqrt(x_dist*x_dist+y_dist*y_dist));
              //if(alpha*int((B1[i][costmap_size/2 -j]+1.5707963268)/alpha/0.0174532925)==h_index[l])
              float partial_angle = B1[i][costmap_size/2 -j]+1.5707963268;
              if ( partial_angle < 0 ) {
                partial_angle += 6.28319;
              }


              if(alpha*int((partial_angle)/alpha/0.0174532925)==h_index[l])
                h[l]+=m1[i][costmap_size/2 -j];
            }
        }
        //ROS_INFO("h %d: %f", h_index[l],h[l]);
      }
    }
    //else if (yaw>= 2.12058 && yaw<= -2.12058 ){
    else{
    ROS_INFO("SECTOR4");
    //else if (sector ==4){
      float B1[costmap_size/2][costmap_size];
      float m1[costmap_size/2][costmap_size];
      //i  da 0 a 150 
      //j da 0 a 299
      for (int l = 0; l< n; l++){
        h_index.push_back(l*alpha);
        h.push_back(0);
        
        for (int i = 0; i< costmap_size/2;i++){
            for(int j = 0; j< costmap_size; j++){
              int cell_cost = costmap_ros_->getCostmap()->getCost(i,j);
              float x_dist = (i-costmap_size/2)*costmap_ros_->getCostmap()->getResolution();
              float y_dist = (j-costmap_size/2)*costmap_ros_->getCostmap()->getResolution();
              B1[i][j]=atan2(y_dist, x_dist);
              //ROS_INFO("B: %f", B1[i-150][j] );
              m1[i][j]=cell_cost*cell_cost * (a1-b1*sqrt(x_dist*x_dist+y_dist*y_dist));
              float partial_angle = B1[i][j] + 1.5707963268;
              if ( partial_angle < 0 ) {
                partial_angle += 6.28319;
              }
              if(alpha*int((partial_angle)/alpha/0.0174532925)==h_index[l])
                h[l]+=m1[i][j];
            }
        }

        //ROS_INFO("h %d: %f", h_index[l],h[l]);
        
      }   
    }
    //ROS_INFO("END SECTORS");
    float target_angle = (atan2(next_pose.pose.position.y -  current_pose.getOrigin().y(),next_pose.pose.position.x -  current_pose.getOrigin().x())+1.5707963268)/0.0174532925;
    ///bug between 270 and 360 deg
    ROS_INFO("TARGET ANGLE CREATED");
    if (target_angle < 0)
      target_angle = 360 + target_angle;
    int min_angle = 5000000;
    int close_angle = 0;
    int l=5;
    vector<float> k;
    k.push_back(h[0]);k.push_back(h[1]);k.push_back(h[2]);k.push_back(h[3]);k.push_back(h[4]);
    for (int i = l; i<n-l ; i++){
      k.push_back((h[i-l]+2*h[i-l+1]+3*h[i-l+2]+4*h[i-l+3]+5*h[i-l+4]+6*h[i]+5*h[i+l-4]+4*h[i+l-3]+3*h[i+l-2]+2*h[i+l-1]+h[i+l])/(2*l+1));
      //ROS_INFO("h %d: %f", h_index[i],k[i]);
    }
    k.push_back(h[h.size()-5]);k.push_back(h[h.size()-4]);k.push_back(h[h.size()-3]);k.push_back(h[h.size()-2]);k.push_back(h[h.size()-1]);
    for (int l = 0; l< n; l++){
            //ROS_INFO("h %d: %f", h_index[l],h[l]);

        //ROS_INFO("h[l]: %f   <  %d    h_free: %d", h[l], threshold, h_index[l]);
	//change k[l] with h[l]
      if (h[l] < threshold){
        if(sector == 1){
            if (h_index[l] > 0 && h_index[l] < 180){
                h_free.push_back(h_index[l]);
            }
        }
        if(sector == 2){
            if (h_index[l] > 90 && h_index[l] < 270){
                h_free.push_back(h_index[l]);
            }
        }
        if(sector == 3){
            //ROS_INFO("SECTOR3 h_index %d", h_index[l]); 
            if (h_index[l] < 90 || h_index[l] > 270){
                h_free.push_back(h_index[l]);
            }
       }
       if(sector == 4){
            if (h_index[l] > 180 && h_index[l] < 360){
                h_free.push_back(h_index[l]);
            }
        }
      }
    }



    //FIND CENTER OF VALLEY
    int cnt = 0;
    vector<int> valley_center;
    for (int l=0; l<h_free.size();l++){
          // ROS_INFO("h_free: %d    %f",h_free[l], h[l]);
    }

    //if (sector == 1){
    if (sector == 1){
      for (int l = 1; l< h_free.size(); l++){
        if(h_free[l-1]+alpha == h_free[l]){
          cnt++;
        }
        else{
          valley_center.push_back(h_free[l-1-cnt/2]);
          //ROS_INFO("cnt: %d", cnt);
          cnt = 0;
        }
        if (l == h_free.size() - 1 ){
          valley_center.push_back(h_free[l-1-cnt/2]);
          //ROS_INFO("cnt: %d", cnt);
          cnt = 0;
          break;        
        }
      }
    }

    //else if (sector==2){
    else if (sector == 2){
      for (int l = 1; l< h_free.size(); l++){
        if ( h_free[l]<90)    //ignore all free angle of the first sector
          continue;
        if(h_free[l-1]+alpha == h_free[l]){
          cnt++;
        }
        else{
          valley_center.push_back(h_free[l-1-cnt/2]);
          //ROS_INFO("cnt: %d", cnt);
          cnt = 0;
        }
        if (l == h_free.size() - 1 ){
          valley_center.push_back(h_free[l-1-cnt/2]);
          //ROS_INFO("cnt: %d", cnt);
          cnt = 0;
          break;        
        }
      }
    }
    //else if (sector==3){
    else if (sector == 3 ){
      int start_condition = 0;
      int end_condition = 0; 
      vector<int> h_free_temp;
      for (int l = 0; l< h_free.size(); l++){
        if (h_free[l]<90){
          start_condition = l;
        }
        else
          break;
      }
      for (int l = 0; l< h_free.size(); l++){
        if (h_free[l]>270){
          end_condition = l;
          break;
        }
      }
      //h_free.erase(h_free.begin(), h_free.end());
      for (int l=start_condition; l>=0; l--){
        h_free_temp.push_back(h_free[l]);
      }
      for (int l=h_free.size()-1; l>=end_condition; l--){
        h_free_temp.push_back(h_free[l]-360);
      }

      h_free = h_free_temp;
      for (int l=0; l< h_free.size(); l++){
        //ROS_INFO("h_free_temp: %d", h_free[l]);
      }

      for (int l = 1; l< h_free.size(); l++){
        if(h_free[l-1]-alpha == h_free[l]){
          cnt++;
        }
        else{
          if (h_free[l-1-cnt/2]>= 0)
            valley_center.push_back(h_free[l-1-cnt/2]);
          else
            valley_center.push_back(h_free[l-1-cnt/2]+360);
          //ROS_INFO("cnt: %d", cnt);
          cnt = 0;
        }/*
        if (h_free[l]==85){
          if (h_free[l-1-cnt/2] >= 0)
            valley_center.push_back(h_free[l-1-cnt/2]);
          else
            valley_center.push_back(h_free[l-1-cnt/2]+360);
          cnt = 0;
        }*/
        if (h_free[l] == -85){
          if (h_free[l-1-cnt/2] >= 0)
            valley_center.push_back(h_free[l-1-cnt/2]);
          else
            valley_center.push_back(h_free[l-1-cnt/2]+360);
          //ROS_INFO("cnt: %d", cnt);
          cnt = 0;
          break;        
        }
      }
    }
    //else if (sector==4){
    /*
    else if (sector_robot == 4 ){
      for (int l = 1; l< h_free.size(); l++){
        if ( h_free[l]<180)    //ignore all free angle of the first and second sector
          continue;
        if(h_free[l-1]+alpha == h_free[l]){
          cnt++;
        }
        else{
          valley_center.push_back(h_free[l-1-cnt/2]);
          //ROS_INFO("cnt: %d", cnt);
          cnt = 0;
        }
        if (h_free[l] == 355){
          valley_center.push_back(h_free[l-1-cnt/2]);
          //ROS_INFO("cnt: %d", cnt);
          cnt = 0;
          break;        
        }
      }
    }*/
    else if (sector == 4 ){
      for (int l = 1; l< h_free.size(); l++){
        if(h_free[l-1]+alpha == h_free[l]){
          cnt++;
        }
        else{
          valley_center.push_back(h_free[l-1-cnt/2]);
          cnt = 0;
        }
        if (l == h_free.size() - 1 ){
          valley_center.push_back(h_free[l-1-cnt/2]);
          cnt = 0;
          break;        
        }
      }
    }



    float h_c = 0; 

    //ROS_INFO("sector: %d", sector);
    for (int l = 0; l< valley_center.size(); l++){
      ROS_INFO("center: %d", valley_center[l]);
      if(abs(target_angle-valley_center[l])< min_angle){
        close_angle = valley_center[l];
        h_c = h[valley_center[l]];
        min_angle = abs(target_angle-valley_center[l]);
      }
    }
    //ROS_INFO("sector: %d  target: %f  close: %d  yaw: %f    h_c: %f", sector, target_angle,  close_angle, yaw, h_c);


    float T = 0.3;
    float tau = 0.4;
    float delta = 0;
    // -90 to compensate the different orientation between the localization system(robot orientation) and start of local costmap. 
    if (sector==1 || sector == 2)
      delta = (close_angle - 90)* 0.0174532925;
    else
      delta = (close_angle - 90)* 0.0174532925;
    
    //float new_delta = (T * delta + (tau - T) * old_delta)/tau;

    if (delta > 3.14159)
      delta = delta -  6.2831853072;
    float new_delta = delta;
    old_delta = delta;
    angular_vel = -yaw + new_delta;
    float delta_temp = delta;
    float yaw_temp = yaw;
    if (sector == 3){ 
	if (delta<0)
		delta_temp = 6.2831853072 + delta;
	if (yaw<0)
		yaw_temp = 6.2831853072 + yaw;
    angular_vel = -yaw_temp + delta_temp;
	/*
	if (new_delta>0)
		angular_vel = yaw - new_delta;
	else
		angular_vel = -yaw + new_delta;
	*/
    }
    else if (sector == 4){ 
	if (yaw > 0 && delta > 0)
    		angular_vel = -yaw + delta;
	else if (yaw > 0 && delta < 0)
    		angular_vel = yaw - delta;
	else if (yaw < 0 && delta > 0)
    		angular_vel = yaw - delta;
	else
    		angular_vel = -yaw + delta;
	
	/*
	if (delta<0)
		delta_temp = 6.2831853072 + delta;
	if (yaw<0)
		yaw_temp = 6.2831853072 + yaw;
	angular_vel = yaw_temp - delta_temp;
	if (new_delta>0)
		angular_vel = yaw - new_delta;
	else
		angular_vel = -yaw + new_delta;
	*/
    }

    angular_vel = (T * angular_vel + (tau - T) * old_angular_vel)/tau;
    old_angular_vel = angular_vel ;
    //angular_vel *= 2;
    ROS_INFO("sector: %d  target: %f  close: %d  yaw: %f    delta_temp: %f   yaw_temp: %f   angula_vel: %f", sector, target_angle,  close_angle, yaw, delta_temp, yaw_temp, angular_vel);
    //ROS_INFO("new_delta: %f", angular_vel);

    float h_param = 50000;
    // if is it needed to turn a lot is better do a rotation in place
    if(abs(angular_vel) < 1.57){
        ROS_INFO("h_c: %f", h_c);
	    linear_vel = 0.2*(1 - (float)std::min(h_c, h_param)/h_param);
    }
    else
	    linear_vel = 0;
    //ROS_INFO("h_c: %f", h_c);
    //ROS_INFO("linear vel: %f", linear_vel);
    ///// END VFH METHOD
    

    /// CHECK IF POSITION REACHED AND ROTATE
    float robot_pos_x = current_pose.getOrigin().x();
    float robot_pos_y = current_pose.getOrigin().y();
    if (sqrt(pow(robot_pos_x - goal_pos_x,2)+pow(robot_pos_y - goal_pos_y,2))< 0.5){
      ROS_INFO("Position reached");
      position_reached = true; 
      linear_vel = 0;
      angular_vel = 0;
    }
    //The robot is in the goal position and need just to rotate to allineate with the goal orientation
    if (position_reached){
      tf::Quaternion quat_goal(
        plan_[plan_.size()-1].pose.orientation.x,
        plan_[plan_.size()-1].pose.orientation.y,
        plan_[plan_.size()-1].pose.orientation.z,
        plan_[plan_.size()-1].pose.orientation.w);
      tf::Matrix3x3 m_goal(quat_goal);
      double roll_goal, pitch_goal, yaw_goal;
      m_goal.getRPY(roll_goal, pitch_goal, yaw_goal);
      linear_vel = 0;
      double P = 0.25;
      angular_vel = P * (yaw_goal - yaw);
      if (abs(angular_vel) < 0.05){
        //ROS_INFO("angular velocity: %f", angular_vel);
        position_reached = false; 
        goal_reached = true;
      }
    }
    else{
    //linear_vel = cos(yaw) * u1 + sin(yaw) * u2;
    //angular_vel = - sin(yaw) / b * u1 + cos(yaw) / b * u2;

    //ROS_INFO("before LINEAR VEL: %f    ANGULAR VEL: %f", linear_vel, angular_vel);
    
    if (linear_vel > 0)
      linear_vel = min(linear_vel, MAX_LIN_VEL);
    else
      linear_vel = max(linear_vel, MIN_LIN_VEL);

    if (angular_vel > 0)
      angular_vel = min(angular_vel, MAX_ANG_VEL);
    else
      angular_vel = max(angular_vel, MIN_ANG_VEL);
    ROS_INFO("linear vel: %f    angular vel: %f", linear_vel, angular_vel);
    
    }
    
    //// END CHECK POSITION REACHED
    //ROS_INFO("yaw: %f", yaw);
    //ROS_INFO("after LINEAR VEL: %f    ANGULAR VEL: %f", linear_vel, angular_vel);



    cmd_vel.linear.x = linear_vel; 
    cmd_vel.linear.y = linear_vel; 
    cmd_vel.linear.z = 0; 

    cmd_vel.angular.z = angular_vel; 

      //ROS_INFO("TEMP: %d",  temp_index);

      //ROS_INFO("COSTMAP VALUE: %d",  costmap_ros_->getCostmap()->getCost(c_x,c_y));
      
    
    /*
    x               x         x
    (299,299)    (299,150) (299,0)


     (0,299)      (0,150)   (0,0)
    x               x         x  
    */ 


  }

  //ROS_INFO("index: %d    x_plan: %f    x_robot: %f",index_to_plan,plan_[index_to_plan].pose.position.x,current_pose.getOrigin().x());
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
  //ROS_INFO("LOCAL PLAN SIZE: %d", plan.size());

  for (int i = 0; i < plan.size(); i++)
    //ROS_INFO("x: %f \t y: %f", plan_[i].pose.position.x, plan_[i].pose.position.y);

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
