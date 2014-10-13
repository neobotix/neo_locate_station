#include <neo_math_toolbox/math_toolbox.h>
#define PI 3.14159265
math_toolbox::math_toolbox(){}
math_toolbox::~math_toolbox(){}
double math_toolbox::calc_dist_point_to_point(geometry_msgs::Point32 p1, geometry_msgs::Point32 p2)
{
	double dist = 0.0;
	double vector_c_x = 0;
	double vector_c_y = 0;
	//X-Wert des neuen Vektors berechnen
	//Startpunkt X wert < 0 und Endpunkt X wert > 0
	if(p1.x < 0 && p2.x > 0)
	{
		vector_c_x = fabs(fabs(p1.x) + fabs(p2.x));
	}
	else if(p1.x > 0 && p2.x < 0) //Startpunkt X wert > 0 und Endpunkt X wert < 0
	{
		vector_c_x = fabs(fabs(p1.x) + fabs(p2.x));
	}
	else //Beide vorzeichen sind gleich
	{
		vector_c_x = fabs(fabs(p1.x) - fabs(p2.x));
	}
	//Y-Wert des neuen Vektors berechnen
	//Startpunkt Y wert < 0 und Endpunkt Y wert > 0
	if(p1.y < 0 && p2.y > 0)
	{
		vector_c_y = fabs(fabs(p1.y) + fabs(p2.y));
	}
	else if(p1.y > 0 && p2.y < 0) //Startpunkt Y wert > 0 und Endpunkt Y wert < 0
	{
		vector_c_y = fabs(fabs(p1.y) + fabs(p2.y));
	}
	else //Beide vorzeichen sind gleich
	{
		vector_c_y = fabs(fabs(p1.y) - fabs(p2.y));
	}
	//Laenge Vektor C berechnen
	dist = sqrt(pow(vector_c_x,2)+pow(vector_c_y,2));
	return dist;
		
}
double math_toolbox::calc_dist_point_to_straight(geometry_msgs::Point32 p1, geometry_msgs::Point32 p2, geometry_msgs::Point32 p3)
{
	geometry_msgs::Point32 p_res;
	double lambda = 0.0;
	double distance_point_to_straight = 0.0;
	double x_k = 0.0;
	double y_k = 0.0;
	double z_k = 0.0;
	double x_c_k = 0.0;
	double y_c_k = 0.0;
	double z_c_k = 0.0;
	
	x_k = p2.x-p1.x;
	y_k = p2.y-p1.y;
	z_k = p2.z-p1.z;

	x_c_k = p1.x-p3.x;
	y_c_k = p1.y-p3.y;
	z_c_k = p1.z-p3.z;

	lambda = ((-1)*((x_c_k*x_k)+(y_c_k*y_k)+(z_c_k*z_k)))/((x_k*x_k)+(y_k*y_k)+(z_k*z_k));
	
	p_res.x = p1.x+(lambda*(p2.x-p1.x));
	p_res.y = p1.y+(lambda*(p2.y-p1.y));
	p_res.z = p1.z+(lambda*(p2.z-p1.z));

	//d = länge des Vektors p_res - p3
	distance_point_to_straight = sqrt(pow((p_res.x-p3.x),2) + pow((p_res.y-p3.y),2) + pow((p_res.z-p3.z),2));
	
	return distance_point_to_straight;
			
}
void math_toolbox::calc_perpendicular_through_point(geometry_msgs::Point32 p1, geometry_msgs::Point32 p2, geometry_msgs::Point32 p3, geometry_msgs::Point32 *p4)
{
	//Bedingung m1 * m2 == -1

	//Gerade durch P(0) und P(End) berechnen.
	//berechne geradengleichung y-y0 = m * (x -x0) = 0 mit m = y1 - y0/x1 -x0
	double m_x = 0;
	double m_y = 0;
	double m = 0;
	double x_koeff = 0;
	double y_koeff = 0;
	double c_koeff = 0;
	
	double m_path = 0;
	double x_koeff_path = 0;
	double y_koeff_path = 0;
	double c_koeff_path = 0;
	
	double x = 0;
	double y = 0;
	double y2 = 0;

	//gedreht, da achsen in der mathematik vertauscht
	m_y = p2.x - p1.x;
	m_x = p2.y - p1.y;
	m = m_y/m_x;
	//berechne werte für die Geradengleichung mit P0
	x_koeff = m;
	y_koeff = -1;
	c_koeff = (m*p1.y)-p1.x;

	//Werte für die Pfadgerade berechnen
	m_path = -1/m;
	x_koeff_path = m_path;
	y_koeff_path = -1;
	c_koeff_path = (m_path*p3.y)-p3.x;
	
	//Berechne Kreuzungspunkt der beiden Gerade
	x = ((c_koeff/y_koeff)-(c_koeff_path/y_koeff_path))/((x_koeff_path/y_koeff_path)-(x_koeff/y_koeff));
	y = ((x * x_koeff)+c_koeff)/y_koeff;
	y2 = ((x * x_koeff_path)+c_koeff_path)/y_koeff_path;
	p4->x = y;
	p4->y = -x;
	p4->z = p3.z;

}
void math_toolbox::calc_point_on_straight_with_given_distance(geometry_msgs::Point32 p1, geometry_msgs::Point32 p2, geometry_msgs::Point32 *p3, double distance)
{
	//           O P2                            P3 O  	_
	//	  ^  |					|	|
	//  P2-P1 |  |	  => Zweipunktform	     P2 O	| distance
	//	  |  |					|	|
	//	     O P1				|	|
	//					     P1 O	_
	double lambda = 0.0;
	
	//lambda aus Punktform und gegebener distanz berechnen
	lambda = sqrt((pow(distance,2)/(pow((p2.x-p1.x),2)+pow((p2.y-p1.y),2)+pow((p2.z-p1.z),2))));

	//lambda in Zwei-Punkt-Form einsetzen
	p3->y = p1.y+(lambda*(p2.y-p1.y)); 
	p3->x = p1.x+(lambda*(p2.x-p1.x)); 
	p3->z = p1.z+(lambda*(p2.z-p1.z)); 
	
	//P3 ist der Punkt auf der Geraden durch P1 und P2 in richtung von P1 nach P2 mit dem Abstand "distance" von P1
}
double math_toolbox::calc_angle_between_two_vectors(geometry_msgs::Point32 p1, geometry_msgs::Point32 p2)
{
	double angle = 0.0; //[°]
	angle = acos(((p1.x*p2.x)+(p1.y*p2.y)+(p1.z*p2.z))/(sqrt(pow(p1.x,2)+pow(p1.y,2)+pow(p1.z,2))*sqrt(pow(p2.x,2)+pow(p2.y,2)+pow(p2.z,2))));
	if(isnan(angle))
	{
		return 0.0;
	}
	if(isinf(angle))
	{
		return 0.0;
	}
	return angle;
}
double math_toolbox::calc_angle_between_vector_and_straight(geometry_msgs::Point32 p1, geometry_msgs::Point32 p2, geometry_msgs::Point32 p3)
{
	//P1 und P2 definieren den resultierenden Vektor P3 definiert den einzelnen vektor
	double angle = 0.0; //[°]
	//richtungsvektor berechnen P2 - P1
	geometry_msgs::Point32 pr;
	pr.x = p1.x-p2.x;
	pr.y = p1.y-p2.y;
	pr.z = p1.z-p2.z;
	angle = acos(((p3.x*pr.x)+(p3.y*pr.y)+(p3.z*pr.z))/(sqrt(pow(p3.x,2)+pow(p3.y,2)+pow(p3.z,2))*sqrt(pow(pr.x,2)+pow(pr.y,2)+pow(pr.z,2))));
	if(isnan(angle))
	{
		return 0.0;
	}
	if(isinf(angle))
	{
		return 0.0;
	}
	if(pr.y < 0)
	{
		return (-1)*angle;
	}
	else
	{
		return angle;
	}
}

