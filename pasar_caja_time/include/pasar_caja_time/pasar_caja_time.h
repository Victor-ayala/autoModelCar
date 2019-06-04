#ifndef SECOND_LAP
#define SECOND_LAP


#include <iostream>
#include <cstdlib>
#include <algorithm>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int16.h>

#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <geometry_msgs/Twist.h>

/*
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
*/
#include <string>


//
//using namespace cv;
using namespace std;

const size_t NUM_DEGREE = 360;

float redondeo(float cantidad, size_t decimales);
float gradosStop(float dist);


class autoModelCar
{
public:
	ros::NodeHandle nh;
	/*
	image_transport::ImageTransport it_;
	image_transport::Subscriber img_sub;
	*/
	//Parametro para k_p (en angle)
	std_msgs::Int16 param_angle;

	//Subscribers
	ros::Subscriber steer_sub;
	ros::Subscriber speed_sub;
	ros::Subscriber laser_sub;
	ros::Subscriber angle_sub;
	ros::Subscriber vel_sub;

	//Publishers
	ros::Publisher speed_pub;
	ros::Publisher steer_pub;

	ros::Publisher obst_pub;

	//Tiempo
	ros::Time ptime, ctime;
	ros::Duration d;

	float laser_distance[NUM_DEGREE];
	/*
	Mat image;
	Mat imgHSV;
	Mat Threshold;
	*/
	sensor_msgs::LaserScan laser_msg;

	//desired_angle
	int dangle;
	int dvel;

	int steer;
	int speed;
	int angle;

	float kp;
	float error;

	//Counters
	int state;

	int obstacle;

	float distanciaF;
	float distanciaI;
	float gradoS;

	double t;
	double tiempoV;

	/* Vector distancia en tiempo, dos valores, uno representa el actual
	y el otro el pasado (count_d = actual)*/
	float distanciaT[3];
	int count_d;

	bool near;


	std_msgs::Int16 steer_16;
	std_msgs::Int16 speed_16;
	std_msgs::Int16 obstacle_16;


public:
	autoModelCar();
	~autoModelCar();
	void lap();

	//Funciones para Suscribirnos
	void laserCallback(const sensor_msgs::LaserScan& laser_P);

	void twistCallback(const geometry_msgs::Twist& twist);

	void speedCallback (const std_msgs::Int16 &speed);

	void angleCallback (const std_msgs::Int16 &angle);
	void velCallback (const std_msgs::Int16 &msg);

	void steerCallback (const std_msgs::Int16 &steer);

	//Regresa la distancia dado un angulo de 0 a 359
	float laserDegree (float degree);

	//Funcion principal para rebasar una caja
	void rebasarCaja();

	//Calcula la distancia, usando el promedio de varios grados
	float distProm(int grado, int num);

	//Pone la distancia actual en distancias[count_d]
	void calcDI(int angulo);
};


#endif
