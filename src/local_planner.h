 /** include the libraries you need in your planner here */
 /** for global path planner interface */
 #include <ros/ros.h>
 #include <costmap_2d/costmap_2d_ros.h>
 #include <costmap_2d/costmap_2d.h>
 #include <nav_core/base_local_planner.h>
 #include <nav_msgs/Path.h>

 #include <geometry_msgs/PoseStamped.h>
 #include <angles/angles.h>
 #include <base_local_planner/world_model.h>
 #include <base_local_planner/costmap_model.h>
#include <vector>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>
#include <Eigen/Dense>

 using std::string;

 #ifndef LOCAL_PLANNER_CPP
 #define LOCAL_PLANNER_CPP

void local_costmap_callback(const nav_msgs::OccupancyGrid &costmap);
 namespace local_planner {

 class LocalPlanner : public nav_core::BaseLocalPlanner {
     bool goal_reached = false; 
     bool position_reached = false; 
     bool initial_allignment = false;

     bool valid_velocity = true;
     bool correct_plan = true; 
     bool start_rotation = false;
     costmap_2d::Costmap2DROS* costmap_ros_;
     std::vector<geometry_msgs::PoseStamped> plan_;
     float x_plan[100];
     float y_plan[100];
     int index_to_plan = 0; 
     int last_plan_point_; 
     float x_pos, y_pos;
     float goal_pos_x, goal_pos_y; 
     std::vector<int> visited_index;

     int cell_x_size, cell_y_size;
     float old_delta, old_linear_vel;

    Eigen::MatrixXd local_costmap_matrixI = Eigen::MatrixXd(300,300);
    Eigen::MatrixXd local_costmap_matrixJ = Eigen::MatrixXd(300,300);
    Eigen::MatrixXd rotation_matrix = Eigen::MatrixXd(300,300);


    void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path);
    void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);
    ros::Publisher g_plan_pub_, l_plan_pub_;

 public:
  int nearestPoint(const int start_point, const tf::Stamped<tf::Pose> & pose);

  bool safety_path(const int next_point, const float yaw);

  void index_calculation_costmap(unsigned int *c_x, unsigned int *c_y, float yaw);

  LocalPlanner();
  LocalPlanner(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS *costmap_ros);

  /** overridden classes from interface nav_core::BaseGlobalPlanner **/
  void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS *costmap_ros);
  bool computeVelocityCommands (geometry_msgs::Twist &cmd_vel);
  bool isGoalReached ();
  bool setPlan (const std::vector< geometry_msgs::PoseStamped > &plan);
  };
 };
 #endif
