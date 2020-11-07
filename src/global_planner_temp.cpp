#include <pluginlib/class_list_macros.h>
#include "global_planner.h"
#include <iostream>
#include <fstream>

 //register this planner as a BaseGlobalPlanner plugin
 PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

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

 //Default Constructor
 namespace global_planner {

 GlobalPlanner::GlobalPlanner (){

 }

 GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
   initialize(name, costmap_ros);
 }

int cost_map[4000][4000];
int path_map[4000][4000];

float res=0.05;
 void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
  ROS_INFO("COSTMAP BRO");
  int x_size = costmap_ros->getCostmap()->getSizeInCellsX();
  int y_size = costmap_ros->getCostmap()->getSizeInCellsY();
  res = costmap_ros->getCostmap()->getResolution();
	for(int i= 0; i<x_size; i++){
		for(int j= 0; j<y_size; j++){
			cost_map[i][j]=costmap_ros->getCostmap()->getCost(i,j);
		}
	}
 }

 bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){
  int cell_goal_x = (goal.pose.position.x+100)/res;
  int cell_goal_y = (goal.pose.position.y+100)/res;
  int cell_start_x = (start.pose.position.x+100)/res;
  int cell_start_y = (start.pose.position.y+100)/res;

  //
  //filter all the cells that are occupied and close to that 
  //
  int border = 1; //4; 
  vector<Point> allowed;
  bool add = true;
  for(int i = 0; i < 4000; i++){
    for(int j = 0; j < 4000; j++){
      if (cost_map[i][j] == 0){
        add = true;
        for(int l = -border; l < border; l++)
          for(int m = -border; m < border; m++)
            if (i+l > 0 && i+l < 4000 && j+m > 0 && j+m < 4000)
              if (cost_map[i+l][j+m] != 0)
                add=false;   
        if(add == true)  
          allowed.push_back(Point(i,j));
      }
    }
  }
 
 //vector of index around start cell
   int step = 10; // 5; 

 vector<Point> ball_start; 
 for (int i = -step ; i< step; i++)
  for (int j = -step ; j< step; j++)
    ball_start.push_back(Point(cell_start_x + i, cell_start_y + j)); 

  vector<Point> index;
  int cell_value = 0;
  index.push_back(Point(cell_goal_x,cell_goal_y));
  int i = 0;
  while (true){
    path_map[index[i].x][index[i].y] = cell_value;
    if (std::find(allowed.begin(), allowed.end(),Point(index[i].x - step, index[i].y))!=allowed.end() && std::find(index.begin(), index.end(),Point(index[i].x -  step, index[i].y))==index.end()){   //check if it is inside the bounderies
      path_map[index[i].x - step][index[i].y] = cell_value + 1;
      index.push_back(Point(index[i].x - step, index[i].y));
    }
    if (std::find(allowed.begin(), allowed.end(),Point(index[i].x + step, index[i].y))!=allowed.end()  && std::find(index.begin(), index.end(),Point(index[i].x + step, index[i].y))==index.end()){                                 //check if it is inside the bounderies
      path_map[index[i].x + step][index[i].y] = cell_value + 1;
      index.push_back(Point(index[i].x + step, index[i].y));
    }
    if (std::find(allowed.begin(), allowed.end(),Point(index[i].x, index[i].y - step ))!=allowed.end() && std::find(index.begin(), index.end(),Point(index[i].x, index[i].y  - step))==index.end()){                                    //check if it is inside the bounderies
      path_map[index[i].x][index[i].y - step] = cell_value + 1;
      index.push_back(Point(index[i].x, index[i].y  - step));
    }
    if (std::find(allowed.begin(), allowed.end(),Point(index[i].x, index[i].y + step))!=allowed.end() && std::find(index.begin(), index.end(),Point(index[i].x, index[i].y  + step))==index.end()){                                 //check if it is inside the bounderies
      path_map[index[i].x][index[i].y + step] = cell_value + 1;
      index.push_back(Point(index[i].x, index[i].y + step));
    }
    cell_value++;

    if (std::find(ball_start.begin(), ball_start.end(),Point(index[i].x, index[i].y))!=ball_start.end())
      break;
    i++;
    //ROS_INFO("I: %d",i);
  }
  
  //
  //find shortest path
  //
  vector<Point> index_path;
  //point from where the robot start
  index_path.push_back(Point(cell_start_x,cell_start_y));
  //point from where the map path is update
  
  //commented to try to remove (keep calling master bug)
  index_path.push_back(Point(index[i].x,index[i].y));

  int cell_value_path = path_map[index[i].x][index[i].y];
  
  i = 1;
  while (cell_value_path>0){
    Point temp(0,0);
    if (path_map[index_path[i].x - step][index_path[i].y]<cell_value_path && std::find(index.begin(), index.end(),Point(index_path[i].x - step,index_path[i].y))!=index.end()){   //check if it is inside the bounderies
      cell_value_path = path_map[index_path[i].x - step][index_path[i].y];
      temp = Point(index_path[i].x - step,index_path[i].y);
    }
    if (path_map[index_path[i].x + step][index_path[i].y]<cell_value_path  && std::find(index.begin(), index.end(),Point(index_path[i].x + step,index_path[i].y))!=index.end()){                                 //check if it is inside the bounderies
      cell_value_path = path_map[index_path[i].x + step][index_path[i].y];
      temp = Point(index_path[i].x + step,index_path[i].y);
    }
    if (path_map[index_path[i].x][index_path[i].y - step]<cell_value_path && std::find(index.begin(), index.end(),Point(index_path[i].x,index_path[i].y-step))!=index.end()){                                    //check if it is inside the bounderies
      cell_value_path = path_map[index_path[i].x][index_path[i].y - step];
      temp = Point(index_path[i].x, index_path[i].y - step );
    }
    if (path_map[index_path[i].x][index_path[i].y + step]<cell_value_path  && std::find(index.begin(), index.end(),Point(index_path[i].x,index_path[i].y + step))!=index.end()){                                 //check if it is inside the bounderies
      cell_value_path = path_map[index_path[i].x][index_path[i].y + step];
      temp = Point(index_path[i].x, index_path[i].y + step);
    }
    index_path.push_back(temp);

    i++;
  }
  
  ROS_INFO("Global path");
  for (int i = 0; i < index_path.size(); i++){
    ROS_INFO("x: %d \t y: %d", index_path[i].x, index_path[i].y);
  }
  
  tf::Quaternion goal_quat;
  plan.push_back(start);

    //orientation

    //get goal yaw   (g_y goal_yaw)
    tf::Quaternion g_y(goal.pose.orientation.x,goal.pose.orientation.y,goal.pose.orientation.z,goal.pose.orientation.w);
    tf::Matrix3x3 m(g_y);
    double roll, pitch, yaw_goal, yaw_current;
    m.getRPY(roll, pitch, yaw_goal);

    //get current yaw (c_y current_yaw)
    tf::Quaternion c_y(start.pose.orientation.x,start.pose.orientation.y,start.pose.orientation.z,start.pose.orientation.w);
    tf::Matrix3x3 m1(c_y);
    m1.getRPY(roll, pitch, yaw_current);

    double yaw_diff = yaw_goal - yaw_current; 
    double increment = yaw_diff/double(index_path.size()); 

  float old_angle = 0; 
  double yaw = yaw_current; 

  //added to reduce the vibration in the local planner (cause in the local planner is not taken the next point but a step is introduced)
  //while (index_path.size()<6)
  //  index_path.push_back(index_path[index_path.size()-1]);

  ROS_INFO("path size: %d",index_path.size());
  for (int i=0; i< index_path.size(); i++){
    geometry_msgs::PoseStamped new_goal = goal;
    yaw+= increment; 
    goal_quat = tf::createQuaternionFromYaw(increment);
    
    new_goal.pose.position.x = (index_path[i].x*res)-100;
    new_goal.pose.position.y = (index_path[i].y*res)-100;

    new_goal.pose.orientation.x = goal_quat.x();
    new_goal.pose.orientation.y = goal_quat.y();
    new_goal.pose.orientation.z = goal_quat.z();
    new_goal.pose.orientation.w = goal_quat.w();

    ROS_INFO("X: %f \t Y: %f", new_goal.pose.position.x, new_goal.pose.position.y);  
    plan.push_back(new_goal);
   }
   plan.push_back(goal);
   return true;
  }
 };
