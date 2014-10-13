//##################
//#### includes ####

//standard includes
//#include <math.h>

//ROS includes
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <laser_geometry/laser_geometry.h>

// ROS message includes
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <geometry_msgs/PolygonStamped.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>


// ROS service includes
#include <std_srvs/Empty.h>

// ROS action lib includes
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

//Special includes
#include <neo_math_toolbox/math_toolbox.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class locator
{
public:
  locator();
  ~locator();

  //Node handle
  ros::NodeHandle n;

  //Service handle
  ros::ServiceServer service;

  //cmd_vel publisher
  ros::Publisher topicPub_vel;

  //publisher
  ros::Publisher topicPub_polygon, topicPub_pose;
  ros::Publisher topicPub_point, topicPub_foot_point, topicPub_first_goal_point;

  // tf listener
  tf::TransformListener listener;

  // laser geometry projector
  laser_geometry::LaserProjection projector;

  //Servicecallback
  bool locateServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  //math_toolbox
  math_toolbox my_math_tb;
  
private:
  bool getparams();
  void LaserScannerCallback(const sensor_msgs::LaserScan::ConstPtr& LS);
  void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom_in);
  int move_to_charging_station(double distance, geometry_msgs::PointStamped *pGoal);
  int move_forward_slow(double distance, geometry_msgs::PointStamped pGoal);
  bool find_station_V1_in_poylgons(std::vector<geometry_msgs::Polygon> *polygons, int *nr_of_polygon, int *nr_of_point);
  bool find_station_V2_in_poylgons(std::vector<geometry_msgs::Polygon> *polygons, int *nr_of_polygon, int *nr_of_point);
  bool convert_laserscan_to_polygons(std::vector<geometry_msgs::Polygon> *polygons);
  bool convert_reflecting_laserscan_to_polygons(std::vector<geometry_msgs::Polygon> *polygons);
  void add_angle_to_quaternion(double angle, geometry_msgs::Quaternion *odom_quat);
  
  ros::Subscriber LaserScan_sub;
  ros::Subscriber Odom_sub;  

  sensor_msgs::LaserScan current_scan_msg;  	    //Current Scan Message
  geometry_msgs::PointStamped act_position;	    //Current Position from Odom

  bool visualize; 

  bool searching_station; //If true discard new incomming laserscans until last search finished
  bool new_laserscan_available;
  bool new_odom_available;

  int nr_of_tries;
  int average_calculation_number;
  int minimal_number_found;
  int version_charging_station;
  double x_threshold, y_threshold;

  int min_points;
  double dist_points_min;
  double dist_points_max;
  double high_min;
  double high_max;

  double vel;

  double dist_pre_goal;
  double dist_goal;

  double curr_roll, curr_pitch, curr_yaw; //Roll Pitch Yaw
};


