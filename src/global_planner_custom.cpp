#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "global_planner_custom");

  ros::NodeHandle n;    

  tf::TransformListener listener;
  while(true){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/map", "/base_scan",
                               ros::Time(0), transform);
      break;
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
  }
  ros::Rate rate(10.0);

     costmap_2d::Costmap2DROS costmap2d_global("global_costmap", listener);
     costmap2d_global.start();
  while (n.ok()){
    costmap2d_global.updateMap();
    int x = costmap2d_global.getCostmap()->getSizeInCellsX();
    int y = costmap2d_global.getCostmap()->getSizeInCellsY();

    int cnt = 0;
    for(int i = 0; i<x; i++){
        for(int j = 0; j<y; j++){
	    cnt++;
            unsigned char cost = static_cast<unsigned>(costmap2d_global.getCostmap()->getCost(i,j));
            std::cout<< static_cast<int>(costmap2d_global.getCostmap()->getCost(i,j));
        }    
    }
    std::cout << std::endl<<cnt;	
    std::cout << std::endl<< std::endl;
     costmap2d_global.stop();
    rate.sleep();
  }
  return 0;


}

