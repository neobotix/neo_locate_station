//##################
//#### includes ####

//standard includes
#include <math.h>

// ROS message includes
#include <geometry_msgs/Point32.h>

class math_toolbox
{
public:
	math_toolbox();
  	~math_toolbox();
	double calc_dist_point_to_point(geometry_msgs::Point32 p1, geometry_msgs::Point32 p2);
  	double calc_dist_point_to_straight(geometry_msgs::Point32 p1, geometry_msgs::Point32 p2, geometry_msgs::Point32 p3);
  	void calc_perpendicular_through_point(geometry_msgs::Point32 p1, geometry_msgs::Point32 p2, geometry_msgs::Point32 p3, geometry_msgs::Point32 *p4);
  	void calc_point_on_straight_with_given_distance(geometry_msgs::Point32 p1, geometry_msgs::Point32 p2, geometry_msgs::Point32 *p3, double distance);
  	double calc_angle_between_two_vectors(geometry_msgs::Point32 p1, geometry_msgs::Point32 p2);
  	double calc_angle_between_vector_and_straight(geometry_msgs::Point32 p1, geometry_msgs::Point32 p2, geometry_msgs::Point32 p3);
private:

};
