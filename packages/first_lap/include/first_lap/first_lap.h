#ifndef FIRST_LAP
#define FIRST_LAP

#include <iostream>
#include <cstdlib>


#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int16.h>

#include <sensor_msgs/LaserScan.h>
#include <cmath>


#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>


//
using namespace cv;
using namespace std;

const size_t NUM_DEGREE = 720;

float redondeo(float cantidad, size_t decimales);
float gradosStop(float dist);


class autoModelCar
{
public:
	ros::NodeHandle nh;
	image_transport::ImageTransport it_;
	image_transport::Subscriber img_sub;

	//Publisher y Subscriber simulacion
	ros::Publisher twist_pub;
	ros::Subscriber laser_sub;

	ros::Subscriber steer_sub;
	ros::Subscriber speed_sub;

	ros::Publisher speed_pub;
	ros::Publisher steer_pub;


	ros::Publisher angle_sub;

	ros::Time ptime, ctime;
	ros::Duration d;

	float laser_distance[NUM_DEGREE];
	int state;
	Mat image;
	Mat imgHSV;
	Mat Threshold;


	//Variables simulacion
	geometry_msgs::Twist twist_msg;
	sensor_msgs::LaserScan laser_msg;

	//desired_angle
	std_msgs::Int16 Dangle_16;
	std_msgs::Int16 steer_16;
	std_msgs::Int16 vel_16;


public:
	autoModelCar();
	~autoModelCar();
	void lap();

	//Funciones para simulacion
	void laserCallback(const sensor_msgs::LaserScan& laser_P);
	void twistCallback(const geometry_msgs::Twist& twist);


	void angleCallback (const std_msgs::Int16 &angle);
	void steerCallback (const std_msgs::Int16 &steer);

	//Regresa la distancia dado un angulo de 0 a 359
	float laserDegree (float degree);

};



#endif
