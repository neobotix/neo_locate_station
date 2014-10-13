#include <neo_locate_station/locate_station.h>

// constructor
locator::locator()
{
	
	getparams();	

	LaserScan_sub = n.subscribe("/sick_s300/scan_filtered", 2, &locator::LaserScannerCallback, this);
	Odom_sub = n.subscribe("/odom", 1, &locator::OdomCallback, this);
	service = n.advertiseService("locate_station", &locator::locateServiceCallback, this);
	// publisher
	if(visualize == true)
	{
	  	topicPub_polygon = n.advertise<geometry_msgs::PolygonStamped>("actual_polygon", 1);
		topicPub_point = n.advertise<geometry_msgs::PointStamped>("goal_point", 1);
		topicPub_foot_point = n.advertise<geometry_msgs::PointStamped>("cross_point", 1);
		topicPub_first_goal_point = n.advertise<geometry_msgs::PointStamped>("pre_goal_point", 1);
		topicPub_pose = n.advertise<geometry_msgs::PoseStamped>("pre_goal_pose", 1);
	}
	//publish velocities
	topicPub_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	
	searching_station = false;
	new_laserscan_available = false;
	
}
// destructor
locator::~locator(){}
bool locator::getparams()
{
	//visualize
	if(n.hasParam("visualize"))
	{
	        n.getParam("visualize", visualize);
	        ROS_INFO("visualize loaded from Parameter-Server.");
	}
	else
	{
		visualize = true;
	        ROS_WARN("No parameter visualize on Parameter-Server. Using default: true");
	}
	//x_threshold
	if(n.hasParam("x_threshold"))
	{
	        n.getParam("x_threshold", x_threshold);
	        ROS_INFO("x_threshold loaded from Parameter-Server is: %f", x_threshold);
	}
	else
	{
		x_threshold = 0.035;
	        ROS_WARN("No parameter x_threshold on Parameter-Server. Using default: %f", x_threshold);
	}
	//y_threshold
	if(n.hasParam("y_threshold"))
	{
	        n.getParam("y_threshold", y_threshold);
	        ROS_INFO("y_threshold loaded from Parameter-Server is: %f", y_threshold);
	}
	else
	{
		y_threshold = 0.035;
	        ROS_WARN("No parameter y_threshold on Parameter-Server. Using default: %f", y_threshold);
	}
	//nr_of_tries
	if(n.hasParam("nr_of_tries"))
	{
	        n.getParam("nr_of_tries", nr_of_tries);
	        ROS_INFO("nr_of_tries loaded from Parameter-Server is: %i", nr_of_tries);
	}
	else
	{
		nr_of_tries = 5;
	        ROS_WARN("No parameter nr_of_tries on Parameter-Server. Using default: %i", nr_of_tries);
	}
	//min_points
	if(n.hasParam("min_points"))
	{
	        n.getParam("min_points", min_points);
	        ROS_INFO("min_points loaded from Parameter-Server is: %i", min_points);
	}
	else
	{
		min_points = 5;
	        ROS_WARN("No parameter min_points on Parameter-Server. Using default: %i", min_points);
	}
	//version
	if(n.hasParam("version"))
	{
	        n.getParam("version", version_charging_station);
	        ROS_INFO("version loaded from Parameter-Server is: %i", version_charging_station);
	}
	else
	{
		version_charging_station = 2;
	        ROS_WARN("No parameter version on Parameter-Server. Using default: %i", version_charging_station);
	}
	//dist_pre_goal
	if(n.hasParam("dist_pre_goal"))
	{
	        n.getParam("dist_pre_goal", dist_pre_goal);
	        ROS_INFO("dist_pre_goal loaded from Parameter-Server is: %f", dist_pre_goal);
	}
	else
	{
		dist_pre_goal = 0.75;
	        ROS_WARN("No parameter dist_pre_goal on Parameter-Server. Using default: %f", dist_pre_goal);
	}
	//dist_goal
	if(n.hasParam("dist_goal"))
	{
	        n.getParam("dist_goal", dist_goal);
	        ROS_INFO("dist_goal loaded from Parameter-Server is: %f", dist_goal);
	}
	else
	{
		dist_goal = 0.48;
	        ROS_WARN("No parameter dist_goal on Parameter-Server. Using default: %f", dist_goal);
	}
	//approach_velocity
	if(n.hasParam("approach_velocity"))
	{
	        n.getParam("approach_velocity", vel);
	        ROS_INFO("approach_velocity loaded from Parameter-Server is: %f", vel);
	}
	else
	{
		vel = 0.05;
	        ROS_WARN("No parameter approach_velocity on Parameter-Server. Using default: %f", vel);
	}
	//dist_points_min
	if(n.hasParam("dist_points_min"))
	{
	        n.getParam("dist_points_min", dist_points_min);
	        ROS_INFO("dist_points_min loaded from Parameter-Server is %f", dist_points_min);
	}
	else
	{
		dist_points_min = 0.38;  
	        ROS_WARN("No parameter dist_points_min on Parameter-Server. Using default: %f", dist_points_min);
	}
	//dist_points_max
	if(n.hasParam("dist_points_max"))
	{
	        n.getParam("dist_points_max", dist_points_max);
	        ROS_INFO("dist_points_max loaded from Parameter-Serverif %f", dist_points_max);
	}
	else
	{
		dist_points_max = 0.41;
	        ROS_WARN("No parameter dist_points_max on Parameter-Server. Using default: %f", dist_points_max);
	}
	//high_min
	if(n.hasParam("high_min"))
	{
	        n.getParam("high_min", high_min);
	        ROS_INFO("high_min loaded from Parameter-Serverif %f", high_min);
	}
	else
	{
		high_min = 0.06;	
	        ROS_WARN("No parameter high_min on Parameter-Server. Using default: %f", high_min);
	}
	//high_max
	if(n.hasParam("high_max"))
	{
	        n.getParam("high_max", high_max);
	        ROS_INFO("high_max loaded from Parameter-Serveris %f", high_max);
	}
	else
	{
		high_max = 0.09;	
	        ROS_WARN("No parameter high_max on Parameter-Server. Using default: %f", high_max);
	}
	//minimal_number_found
	if(n.hasParam("min_nr_found"))
	{
	        n.getParam("min_nr_found", minimal_number_found);
	        ROS_INFO("min_nr_found loaded from Parameter-Serveris %i", minimal_number_found);
	}
	else
	{
		minimal_number_found = 12;	
	        ROS_WARN("No parameter min_nr_found on Parameter-Server. Using default: %i", minimal_number_found);
	}
	//average_calculation_number
	if(n.hasParam("average_calculation_number"))
	{
	        n.getParam("average_calculation_number", average_calculation_number);
	        ROS_INFO("average_calculation_number loaded from Parameter-Serveris %i", average_calculation_number);
	}
	else
	{
		average_calculation_number = 40;	
	        ROS_WARN("No parameter average_calculation_number on Parameter-Server. Using default: %i", average_calculation_number);
	}
	return true;
}
//Odom Callback
void locator::OdomCallback(const nav_msgs::Odometry::ConstPtr& odom_in)
{
	//Get act position
	act_position.point.x = odom_in->pose.pose.position.x;
	act_position.point.y = odom_in->pose.pose.position.y;
	act_position.point.z = odom_in->pose.pose.position.z;
	new_odom_available = true;
	//Get act RPY
	tf::Quaternion q(odom_in->pose.pose.orientation.x, odom_in->pose.pose.orientation.y, odom_in->pose.pose.orientation.z, odom_in->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(curr_roll, curr_pitch, curr_yaw);
}

//Laserscanner Callback
void locator::LaserScannerCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	if(!searching_station)
	{
		current_scan_msg = *scan_in;
		new_laserscan_available = true;
	}
}

//Service Callback
bool locator::locateServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	int state = 0;
	searching_station = false;
	ROS_INFO("ServiceCalled!");
	int tries = 0;
	geometry_msgs::PointStamped pGoal;
	do
	{
		state = move_to_charging_station(dist_pre_goal, &pGoal);
		switch(state)
		{
			case 0: ROS_INFO("charging station not found"); break;
			case 1: ROS_INFO("calculation error"); break;
			case 2: ROS_INFO("goal not reached or timeout (30sec)"); break;
			case 3: ROS_INFO("goal reached"); break;
			case 90: ROS_ERROR("Version not set!!"); tries = nr_of_tries+1; break;
			case 99: ROS_ERROR("Transformation error"); break;
		}
		tries++;
		if(tries >= nr_of_tries)
		{
			ROS_INFO("max. tries reached quit!");
			return false;
		}
	}while(state != 3);

	state = move_forward_slow(dist_goal, pGoal);
	if(state == 4)
	{
		return true;
	}
	else
	{
		return false;
	}
}
int locator::move_forward_slow(double distance, geometry_msgs::PointStamped pGoal)
{
	
	geometry_msgs::Twist cmd_vel;
	double act_distance = 0.0;

	//publish cmd_vel
	//linear vel
	cmd_vel.linear.x = vel;
	cmd_vel.linear.y = 0.0;
	cmd_vel.linear.z = 0.0;
	//angular vel
	cmd_vel.angular.x = 0.0;
	cmd_vel.angular.y = 0.0;
	cmd_vel.angular.z = 0.0;
	topicPub_vel.publish(cmd_vel);
	if(visualize == true)
	{
		topicPub_foot_point.publish(pGoal);
	}
	do
	{
		//move forward, until distance between actual position and goal is <= distance
		//get actual position from odom
		new_odom_available = false;
		do
		{
			ros::spinOnce();
		}while(new_odom_available == false);
	
		//calculate distance between these two vectors
		geometry_msgs::Point32 p1_tmp; //convert geometry_msgs::PointStamped in geometry_msgs::Point32, because calc_dist_point_to_point accepts only geometry_msgs::Point32!
		geometry_msgs::Point32 p2_tmp;
		//P1
		p1_tmp.x = pGoal.point.x;
		p1_tmp.y = pGoal.point.y;
		p1_tmp.z = pGoal.point.z;
		//P2
		p2_tmp.x = act_position.point.x;
		p2_tmp.y = act_position.point.y;
		p2_tmp.z = pGoal.point.z; //set both points to same high
		act_distance = my_math_tb.calc_dist_point_to_point(p1_tmp, p2_tmp);
		//ROS_INFO("act_dist: %f", act_distance);
		if(visualize == true)
		{
			act_position.header.stamp = ros::Time::now();
			act_position.header.frame_id = "/odom";
			topicPub_foot_point.publish(act_position);
		}
	
	}while(act_distance > distance);
	//publish cmd_vel
	//linear vel
	cmd_vel.linear.x = 0.0;
	cmd_vel.linear.y = 0.0;
	cmd_vel.linear.z = 0.0;
	//angular vel
	cmd_vel.angular.x = 0.0;
	cmd_vel.angular.y = 0.0;
	cmd_vel.angular.z = 0.0;
	topicPub_vel.publish(cmd_vel);
	if(act_distance <= distance)
	{
		return 4;
	}
	else
	{
		return 5;
	}
	
}
int locator::move_to_charging_station(double distance, geometry_msgs::PointStamped *pGoal)
{
	//states:
	//0 = Charging statoin not found in polygons
	//1 = Calculation error
	//2 = Goal not reached at all or not reached in 30sec
	//3 = Goal reached!
	//90 = Version not set!
	int state = 0;

	//Search Polygon-------------------------------
	std::vector<geometry_msgs::Polygon> polygons;
	geometry_msgs::Point32 pA;
	pA.x = 0.0;
	pA.y = 0.0;
	pA.z = 0.0;
	geometry_msgs::Point32 pB;
	pB.x = 0.0;
	pB.y = 0.0;
	pB.z = 0.0;
	geometry_msgs::Point32 pC;
	pC.x = 0.0;
	pC.y = 0.0;
	pC.z = 0.0;	
	bool found_polygon = false;
	int nr_times_found = 0;
	int nr_of_found_polygon = 0;
	int nr_of_point = 0;
	//--------------------------------------------
	//calculate path--------------------------------
	geometry_msgs::Point32 cross_point;
	geometry_msgs::Point32 goal_point;
	double angle = 0.0; //[°]
	geometry_msgs::Quaternion odom_quat;
	geometry_msgs::PointStamped pC_odom;
	//unity vector in X
	geometry_msgs::Point32 e_x;
	e_x.x = 1;
	e_x.y = 0;
	e_x.z = 0;
	//---------------------------------------------
	//drive to goal--------------------------------
	move_base_msgs::MoveBaseGoal goal;

	MoveBaseClient ac("move_base", true);
	ROS_INFO("Waiting for move_base server...");
	ac.waitForServer();
	ROS_INFO("move_base server online!");

	//---------------------------------------------
//1. find station in polygons
//1.1 sum points
	for(int e = 0; e < average_calculation_number; e++)
	{	
		new_laserscan_available = false;
		searching_station = false;
		do
		{
			ros::Duration(0.01).sleep();
			ros::spinOnce();
		}while(new_laserscan_available == false);
		searching_station = true;
		if(new_laserscan_available == true)
		{
			switch(version_charging_station)
			{
				case 1: found_polygon = find_station_V1_in_poylgons(&polygons, &nr_of_found_polygon, &nr_of_point); break;
				case 2: found_polygon = find_station_V2_in_poylgons(&polygons, &nr_of_found_polygon, &nr_of_point); break;
				default: return 90;
			}
			if(found_polygon == true)
			{
				pA.x += polygons.at(nr_of_found_polygon).points.at(0).x;
				pA.y += polygons.at(nr_of_found_polygon).points.at(0).y;
				pA.z += polygons.at(nr_of_found_polygon).points.at(0).z;
			
				pB.x += polygons.at(nr_of_found_polygon).points.at(polygons.at(nr_of_found_polygon).points.size()-1).x;
				pB.y += polygons.at(nr_of_found_polygon).points.at(polygons.at(nr_of_found_polygon).points.size()-1).y;
				pB.z += polygons.at(nr_of_found_polygon).points.at(polygons.at(nr_of_found_polygon).points.size()-1).z;

				pC.x += polygons.at(nr_of_found_polygon).points.at(nr_of_point).x;
				pC.y += polygons.at(nr_of_found_polygon).points.at(nr_of_point).y;
				pC.z += polygons.at(nr_of_found_polygon).points.at(nr_of_point).z;
				nr_times_found++;
			}
		}
		else
		{
			ROS_WARN("Spinning but no new scan!!!");
		}
		
	}
//1.1 calc average point coordinates	
	pA.x = pA.x/nr_times_found;
	pA.y = pA.y/nr_times_found;
	pA.z = pA.z/nr_times_found;

	pB.x = pB.x/nr_times_found;
	pB.y = pB.y/nr_times_found;
	pB.z = pB.z/nr_times_found;

	pC.x = pC.x/nr_times_found;
	pC.y = pC.y/nr_times_found;
	pC.z = pC.z/nr_times_found;
	//ROS_INFO("times found: %i", nr_times_found);
//2. calc point in given distance
	if(nr_times_found >= minimal_number_found)
	{
		state = 1;
		//wenn die Ladestation gefunden wurde berechne die bahn.
		
		//2.1 berechne Gerade senkrecht auf der Geraden aus P(0) und P(End) und durch P(max_dist)
		//Benötigte Parameter P1, P2, P3
		//Rückgabe vom Kreuzungspunkt als Pointer P4
		//calc_perpendicular_through_point(polygons.at(nr_of_found_polygon).points.at(0), polygons.at(nr_of_found_polygon).points.at(polygons.at(nr_of_found_polygon).points.size()-1), polygons.at(nr_of_found_polygon).points.at(nr_of_point), &cross_point);
		my_math_tb.calc_perpendicular_through_point(pA, pB, pC, &cross_point);
		//2.2 Bereche Punkt vor Ladestation und verdrehung, um diese als Goal an Move_Base zu senden
		//Berechne Punkt 75 cm vor Eckpunkt
		//calc_point_on_straight_with_given_distance(polygons.at(nr_of_found_polygon).points.at(nr_of_point), cross_point, &goal_point, distance);
		my_math_tb.calc_point_on_straight_with_given_distance(pC, cross_point, &goal_point, distance);
		//Vergleiche den abstand des errechneten Punktes mit dem abstand zu pC
		//	->der Abstand zum errechneten Punkt darf nicht größer als der zu pC sein,
		//	  sonst liegt der Punkt auf der falschen seite
		// transform act_position from pointstamped to point32
		//geometry_msgs::Point32 p_act_tmp;
		//p_act_tmp.x = act_position.point.x;
		//p_act_tmp.y = act_position.point.y;
		//p_act_tmp.z = pC.z; //damit beide auf gleicher höhe sind
		//ROS_INFO("pC.x = %f ------  pC.y = %f",pC.x,pC.y);
		//ROS_INFO("goal_point.x = %f ------  goal_point.y = %f",goal_point.x,goal_point.y);
		//ROS_INFO("Dist Act to goal: %f", sqrt(pow(goal_point.x,2)+pow(goal_point.y,2)));
		//ROS_INFO("Dist Act to pC: %f", sqrt(pow(pC.x,2)+pow(pC.y,2)));
		if(sqrt(pow(goal_point.x,2)+pow(goal_point.y,2)) < sqrt(pow(pC.x,2)+pow(pC.y,2)))
		{
			//ROS_INFO("Dist Act to goal: %f", my_math_tb.calc_dist_point_to_point(p_act_tmp, goal_point));
                	//ROS_INFO("Dist Act to pC: %f", my_math_tb.calc_dist_point_to_point(p_act_tmp, pC));
		}
		else
		{
			ROS_ERROR("Point on wrong side: ");
			my_math_tb.calc_point_on_straight_with_given_distance(cross_point, pC, &goal_point, distance);
		}
		
		//calculate orientation
		angle = my_math_tb.calc_angle_between_vector_and_straight(pC, goal_point, e_x);	
		//2.3 calculate quaternion from given angles
		add_angle_to_quaternion(angle, &odom_quat);
		
		state = 2;
		//transform pGoal from /base_link frame to /odom frame
		pC_odom.header.stamp = ros::Time::now();
                pC_odom.header.frame_id = "/base_link";
                pC_odom.point.x = pC.x;
                pC_odom.point.y = pC.y;
                pC_odom.point.z = 0.0;
                
                try
                {
                        //wait for transform from base_link to odom
                        listener.waitForTransform("/odom", pC_odom.header.frame_id, pC_odom.header.stamp, ros::Duration(3.0));
                        //transform
                        listener.transformPoint("/odom", pC_odom, *pGoal);

                }
                catch(tf::TransformException ex){
                        ROS_ERROR("%s",ex.what());
                        return 99;
                }
		//Publish-------------------------------------------------------
		if(visualize == true)
		{
			//Publishing -----------
			geometry_msgs::PolygonStamped poly_stamp;
			geometry_msgs::PointStamped point_stamp;
			geometry_msgs::PointStamped foot_point_stamp;
			geometry_msgs::PointStamped first_goal_point_stamp;
			geometry_msgs::PoseStamped pose_stamp;
			//----------------------
			ROS_INFO("Polygon found: %i", found_polygon);
			ROS_INFO("Polygon found: %i",nr_of_found_polygon);
			ROS_INFO("Point found: %i",nr_of_point);

			//Goalpose-------------------------------------
			pose_stamp.header.stamp = ros::Time::now();
			pose_stamp.header.frame_id = "/base_link";
			pose_stamp.pose.position.x = goal_point.x;
			pose_stamp.pose.position.y = goal_point.y;
			pose_stamp.pose.position.z = goal_point.z;
			pose_stamp.pose.orientation.x = odom_quat.x;
			pose_stamp.pose.orientation.y = odom_quat.y;
			pose_stamp.pose.orientation.z = odom_quat.z;
			pose_stamp.pose.orientation.w = odom_quat.w;
			topicPub_pose.publish(pose_stamp);
			//---------------------------------------------------	

			//CROSS-Point-------------------------------------
			foot_point_stamp.header.stamp = ros::Time::now();
			foot_point_stamp.header.frame_id = "/base_link";
			foot_point_stamp.point.x = cross_point.x;
			foot_point_stamp.point.y = cross_point.y;
			foot_point_stamp.point.z = polygons.at(0).points.at(0).z;
			topicPub_foot_point.publish(foot_point_stamp);
			//---------------------------------------------------

			//Goal-Point-------------------------------------
			first_goal_point_stamp.header.stamp = ros::Time::now();
			first_goal_point_stamp.header.frame_id = "/base_link";
			first_goal_point_stamp.point.x = goal_point.x;
			first_goal_point_stamp.point.y = goal_point.y;
			first_goal_point_stamp.point.z = goal_point.z;
			topicPub_first_goal_point.publish(first_goal_point_stamp);
			//---------------------------------------------------

			//Most-Dist-Point-------------------------------------
			point_stamp.header.stamp = ros::Time::now();
			point_stamp.header.frame_id = "/base_link";
			point_stamp.point.x = pC.x;
			point_stamp.point.y = pC.y;
			point_stamp.point.z = pC.z;	
			topicPub_point.publish(point_stamp);
			//---------------------------------------------------

			//Polygon---------------------------------------------
			poly_stamp.header.stamp = ros::Time::now();
			poly_stamp.header.frame_id = "/base_link";
			poly_stamp.polygon = polygons.at(nr_of_found_polygon);
			topicPub_polygon.publish(poly_stamp);
			//---------------------------------------------------
		}
		//----------------------------------------------------------------------
//3. publish Goal and wait for move_base to get there
		//3.1 create Goal
		goal.target_pose.header.frame_id = "/base_link";
		goal.target_pose.header.stamp = ros::Time::now();
		goal.target_pose.pose.position.x = goal_point.x;
		goal.target_pose.pose.position.y = goal_point.y;
		goal.target_pose.pose.position.z = 0.0;
		goal.target_pose.pose.orientation = odom_quat;

		//3.2 send Goal	
		ROS_INFO("Send goal");
		ac.sendGoal(goal);
               	ROS_INFO("Waiting.... (max. 30sec)");
		ac.waitForResult(ros::Duration(30.0));
	
  		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			state = 3;
		}
		
		
	}
	//4. return state
	
	return state;
}
void locator::add_angle_to_quaternion(double angle, geometry_msgs::Quaternion *odom_quat)
{
	//get actual orientation from odom
	new_odom_available = false;
	do
	{
		ros::spinOnce();
	}while(new_odom_available == false);
	
	geometry_msgs::Quaternion odom_geo_tmp;
	tf::Quaternion odom_tmp(tf::createQuaternionFromRPY(curr_roll, curr_pitch, angle));
	tf::quaternionTFToMsg(odom_tmp, odom_geo_tmp);
	odom_quat->x = odom_geo_tmp.x;
	odom_quat->y = odom_geo_tmp.y;
	odom_quat->z = odom_geo_tmp.z;
	odom_quat->w = odom_geo_tmp.w;
	
}
bool locator::find_station_V2_in_poylgons(std::vector<geometry_msgs::Polygon> *polygons, int *nr_of_polygon, int *nr_of_point)
{
	//transform laserscan to pointcloud and collect points to polygons
	convert_reflecting_laserscan_to_polygons(polygons);
	
	//Prüfe jedes Polygon, ob es unser gesuchtes sein kann.
	/*Vorgehensweise:	1. Prüfe ob das aktuelle Polygon und das folgende Polygon jeweils mindestens 5 punkte enthalten
				2. Berechne Abstand zwischen den gemittelten Reflektorpunkten + Prüfe diesen Abstand
				3. Berechne Mittelpunkt zwischen den gemittelten Reflektorpunkten
	*/
	double dist_point_to_point = 0.0;
	geometry_msgs::Polygon act_polygon;
	geometry_msgs::Point32 p_m; //middle
	geometry_msgs::Point32 p_s; //start
	geometry_msgs::Point32 p_e; //end
	for(int z = 0; z < polygons->size();z++)
	{
		act_polygon.points.clear();
		//Check if act polygon contains more than min_points points
		if(polygons->at(z).points.size() > min_points)
		{
			if(visualize == true)
			{
				//Polygon---------------------------------------------
				geometry_msgs::PolygonStamped poly_stamp;
				poly_stamp.header.stamp = ros::Time::now();
				poly_stamp.header.frame_id = "/base_link";

				poly_stamp.polygon = polygons->at(z);
			
				topicPub_polygon.publish(poly_stamp);
				//-----------------------------------------------------
			}
			//is there a next polygon?
			if(z+1 < polygons->size())
			{
				if(polygons->at(z+1).points.size() > min_points)
				{
					//Check distance beteween these two points
					dist_point_to_point = my_math_tb.calc_dist_point_to_point(polygons->at(z).points.at(polygons->at(z).points.size()-1), polygons->at(z+1).points.at(0));
					//ROS_INFO("Act. Dist: %f", dist_point_to_point);
					if(dist_point_to_point < dist_points_max && dist_point_to_point > dist_points_min)
					{
						//ROS_INFO("das ist unser polygon");
						//berechne mittelpunkt zwischen den beiden abstandspunkten
						my_math_tb.calc_point_on_straight_with_given_distance(polygons->at(z).points.at(polygons->at(z).points.size()-1), polygons->at(z+1).points.at(0), &p_m, (dist_point_to_point/2));
						p_e = polygons->at(z+1).points.at(polygons->at(z+1).points.size()-1);
						p_s = polygons->at(z).points.at(0);
						//delete all old polygons
						polygons->clear();
						//store points in polygons
						act_polygon.points.push_back(p_s);
						act_polygon.points.push_back(p_m);
						act_polygon.points.push_back(p_e);
						polygons->push_back(act_polygon);
						*nr_of_polygon = 0;
						*nr_of_point = 1;
						return true;
					}
				}
			}
			
		}
	}
	return false;
	
}
bool locator::find_station_V1_in_poylgons(std::vector<geometry_msgs::Polygon> *polygons, int *nr_of_polygon, int *nr_of_point)
{
	//transform laserscan to pointcloud and collect points to polygons
	convert_laserscan_to_polygons(polygons);

	//Prüfe jedes Polygon, ob es unser gesuchtes sein kann.
	/*Vorgehensweise:	1. Berechne Abstand zwischen Start und Endpunkt + Prüfe diesen Abstand
				2. Berechne Punkt im Polygon, der am weitesten von der Geraden durch Start und Endpunkt entfernt ist. + Prüfe diesen Abstand
	*/

	double dist_point_to_point = 0.0;
	double distance_point_to_straight = 0.0;
	double last_dist = 0.0;
	double dist_straight_to_farest_point = 0.0;
	
	bool found_polygon = false;
	
	
	for(int z = 0; z < polygons->size();z++)
	{
		//Check if act polygon contains more than min_points points
		if(polygons->at(z).points.size() > min_points)
		{
			//Check distance beteween these two points
			dist_point_to_point = my_math_tb.calc_dist_point_to_point(polygons->at(z).points.at(0), polygons->at(z).points.at(polygons->at(z).points.size()-1));		
			//ROS_INFO("Distance Point to Point: %f", dist_point_to_point);
			if(dist_point_to_point < dist_points_max && dist_point_to_point > dist_points_min)
			{
				//ROS_INFO("Passende distanz in polygon %i gefunden. Abstand: %f", z, dist_point_to_point);
				*nr_of_polygon = z;
				if(visualize == true)
				{
					//Polygon---------------------------------------------
					geometry_msgs::PolygonStamped poly_stamp;
					poly_stamp.header.stamp = ros::Time::now();
					poly_stamp.header.frame_id = "/base_link";

					poly_stamp.polygon = polygons->at(z);
			
					topicPub_polygon.publish(poly_stamp);
					//-----------------------------------------------------	
				}
				dist_straight_to_farest_point = 0.0;
				last_dist = 0.0;
				//find point in polygon which is most fare away from straight
				for(int g = 0; g < polygons->at(z).points.size(); g++)
				{	
					distance_point_to_straight = my_math_tb.calc_dist_point_to_straight(polygons->at(z).points.at(0), polygons->at(z).points.at(polygons->at(z).points.size()-1), polygons->at(z).points.at(g));
					//ROS_INFO("High: %f", distance_point_to_straight);
					if(distance_point_to_straight > last_dist)
					{
						last_dist = distance_point_to_straight;
						dist_straight_to_farest_point = distance_point_to_straight;
						*nr_of_point = g;
					}
				}
				//Check distance beteween point and straight
				if(dist_straight_to_farest_point > dist_points_min && dist_straight_to_farest_point < dist_points_max) 
				{
					if(visualize == true)
					{
						//Most-Dist-Point-------------------------------------
						geometry_msgs::PointStamped point_stamp;
						point_stamp.header.stamp = ros::Time::now();
						point_stamp.header.frame_id = "/base_link";
						point_stamp.point.x = polygons->at(z).points.at(*nr_of_point).x;
						point_stamp.point.y = polygons->at(z).points.at(*nr_of_point).y;
						point_stamp.point.z = polygons->at(0).points.at(0).z;	
						topicPub_point.publish(point_stamp);
						//---------------------------------------------------
					}
					found_polygon = true;
					return found_polygon;
					//This is the polygon.
				}
				
			}
			
		}
	}
	return found_polygon;
}
//Find reflecting surfaces in the laserscan and combine them to polygons
bool locator::convert_reflecting_laserscan_to_polygons(std::vector<geometry_msgs::Polygon> *polygons)
{
	int nr_of_polygon = 0;
	sensor_msgs::PointCloud cloud;
	geometry_msgs::Polygon act_polygon;
	geometry_msgs::Point32 act_point;

	polygons->clear();
	cloud.points.clear();
	cloud.channels.clear();
	//Header
	cloud.header.stamp = current_scan_msg.header.stamp;
	//Convert Laserscan to Pointcloud
	try
      	{
		//transform from laserframe to base_link
        	listener.waitForTransform("/base_link", current_scan_msg.header.frame_id, current_scan_msg.header.stamp, ros::Duration(3.0));

        	//ROS_DEBUG("now project to point_cloud");
        	projector.transformLaserScanToPointCloud("/base_link", current_scan_msg, cloud, listener);
      	}
      	catch(tf::TransformException ex){
        	ROS_ERROR("%s",ex.what());
		return false;
      	}
	int nr_channel = 0; //we only have an intensity channel so its 0
	int anz_points = cloud.points.size();
	double default_intensity = 0.0;
	int nr_points = 0;
	//ROS_INFO("default intensity: %f", default_intensity);
	for(int i = 0; i < anz_points; i++)
	{
		if(cloud.channels[nr_channel].values[i] != default_intensity)
		{
			act_point.x = cloud.points[i].x;
			act_point.y = cloud.points[i].y;
			act_point.z = cloud.points[i].z;
			act_polygon.points.push_back(act_point);
			nr_points++;
			
		}
		else
		{
			if(nr_points > 0)
			{
				//maybe new object comming
				//save old polygon
				polygons->push_back(act_polygon);
				nr_of_polygon++;
				act_polygon.points.clear();
				nr_points = 0;
			}
			else
			{
				act_polygon.points.clear();
			}
		}
	}
	
	return true;
}
//Combine points with a given distance to each other to polygons
bool locator::convert_laserscan_to_polygons(std::vector<geometry_msgs::Polygon> *polygons)
{

	int nr_of_polygon = 0;
	sensor_msgs::PointCloud cloud;
	geometry_msgs::Polygon act_polygon;
	geometry_msgs::Point32 act_point;

	polygons->clear();
	cloud.points.clear();
	cloud.channels.clear();
	//Header
	cloud.header.stamp = current_scan_msg.header.stamp;
	//Convert Laserscan to Pointcloud
	try
      	{
		//transform from laserframe to base_link
        	listener.waitForTransform("/base_link", current_scan_msg.header.frame_id, current_scan_msg.header.stamp, ros::Duration(3.0));

        	//ROS_DEBUG("now project to point_cloud");
        	projector.transformLaserScanToPointCloud("/base_link", current_scan_msg, cloud, listener);
      	}
      	catch(tf::TransformException ex){
        	ROS_ERROR("%s",ex.what());
		return false;
      	}
	//seperate points in cloud to geometry_msgs/Polygon
	int anz_points = cloud.points.size();
	int nr_of_point_in_polygon = 0;
	double act_distance_point_to_point_x = 0;
	double act_distance_point_to_point_y = 0;
	double act_distance_point_to_point_z = 0;
	//ROS_INFO("Anz. Points: %i", anz_points);
	for(int i = 0; i < anz_points; i++)
	{
		//If its the first Point create the first polygon with this point
		if(i == 0)
		{
			//create a new polygon object
			//polygons.push_back();
			act_point.x = cloud.points[i].x;
			act_point.y = cloud.points[i].y;
			act_point.z = cloud.points[i].z;
			act_polygon.points.push_back(act_point);
			nr_of_point_in_polygon++;
		}
		else
		{
			//Check if current point is near to last point
			act_distance_point_to_point_x = fabs(fabs(cloud.points[i].x) - fabs(act_polygon.points[nr_of_point_in_polygon-1].x));
			act_distance_point_to_point_y = fabs(fabs(cloud.points[i].y) - fabs(act_polygon.points[nr_of_point_in_polygon-1].y));
			act_distance_point_to_point_z = fabs(fabs(cloud.points[i].z) - fabs(act_polygon.points[nr_of_point_in_polygon-1].z));
			if(act_distance_point_to_point_x < x_threshold)
			{
				//point is close to next point in x und y => same object
				if(act_distance_point_to_point_y < y_threshold)
				{
					act_point.x = cloud.points[i].x;
					act_point.y = cloud.points[i].y;
					act_point.z = cloud.points[i].z;
					act_polygon.points.push_back(act_point);
					nr_of_point_in_polygon++;
				}
				else //point is too fare away from next point in y => new object (polygon)
				{
					//save old polygon 
					polygons->push_back(act_polygon);
					nr_of_polygon++;
					nr_of_point_in_polygon = 0;
					act_polygon.points.clear();
					//set first point in new polygon
					act_point.x = cloud.points[i].x;
					act_point.y = cloud.points[i].y;
					act_point.z = cloud.points[i].z;
					act_polygon.points.push_back(act_point);
					nr_of_point_in_polygon++;
				}
			}
			else //point is too fare away from next point in x => new object (polygon)
			{
				//altes polygon speichern
				polygons->push_back(act_polygon);
				nr_of_polygon++;
				nr_of_point_in_polygon = 0;
				act_polygon.points.clear();
				//set first point in new polygon
				act_point.x = cloud.points[i].x;
				act_point.y = cloud.points[i].y;
				act_point.z = cloud.points[i].z;
				act_polygon.points.push_back(act_point);
				nr_of_point_in_polygon++;
			}
		}
	}
	polygons->push_back(act_polygon);
	nr_of_polygon++;
	nr_of_point_in_polygon = 0;
	act_polygon.points.clear();
	return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "locate_station_node");
  locator loc_station;
  
  ROS_INFO("Ready to search for stations!");
  ros::Rate loop_rate(20); // Hz 
  while(loc_station.n.ok())
  {
	loop_rate.sleep();
	ros::spinOnce();
  }

  return 0;
}
