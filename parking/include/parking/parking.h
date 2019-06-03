#ifndef PARKING
#define PARKING


#include <iostream>
#include <cstdlib>
#include <algorithm>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int16.h>

#include <sensor_msgs/LaserScan.h>
#include <cmath>

#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>
#include <string>

using namespace std;

const size_t NUM_DEGREE = 360;

class autoModelCar {
public:
	ros::NodeHandle nh;

	//Parametro para k_p (en angle)
	std_msgs::Int16 param_angle;

	//Subscribers
	ros::Subscriber steer_sub;
	ros::Subscriber speed_sub;
	ros::Subscriber laser_sub;
	ros::Subscriber angle_sub;

	//Publishers
	ros::Publisher speed_pub;
	ros::Publisher steer_pub;

	//Tiempo
	ros::Time ptime, ctime;
	ros::Duration d;

	// Laser distance buffer [360 elements]
	float laser_distance[NUM_DEGREE];
	
	sensor_msgs::LaserScan laser_msg;

	//desired_angle
	int dangle;

	int steer;
	int speed;
	int angle;

	float kp;
	float error;

	//Counters
	int state;

	std_msgs::Int16 steer_16;
	std_msgs::Int16 speed_16;


public:
	autoModelCar();
	~autoModelCar();

	//Funciones para simulacion
	void laserCallback(const sensor_msgs::LaserScan& laser_P);
	void twistCallback(const geometry_msgs::Twist& twist);

	void speedCallback (const std_msgs::Int16 &speed);

	void angleCallback (const std_msgs::Int16 &angle);

	void steerCallback (const std_msgs::Int16 &steer);

	//Funcion para pausar
	void Pause(double seconds);

	// Erick
	// parking: Funcion para estacionar con condiciones de lidar
	void  parking();
	// box_detect: Detecta caja con diferentes parametros
	// [angle: angulo de lidar deseado] [leq: distance less or equal than][tol: tolerance
	// fbox_detect: Fake box_detect (pruebas sin amc)
	int box_detect(int angle, double leq, double tol);	
	int fbox_detect(int angle,double leq, double tol);	

};

#endif
