/*
 * First lap autoModelCar
 *
 * Authors:	Victor Ayala Alfaro
 *		Ricardo Buena Corona
 *		Erick Aguilera Camacho
 *		Emilio Puga Avalos
 * 			Vision, Robotics and Artificial Intelligence Laboratory
 * 			LaViRIA
 *
 * 03/2018
 *
 * */

#include "parking.h"

using namespace std;

double flaser[360]={0.0};	// fake laser data
int c = 0;	// contador
int bs;	// Box status

autoModelCar::autoModelCar():nh("~")
{
	ROS_INFO("Init parking");

	//Publicar
	speed_pub = nh.advertise<std_msgs::Int16>("/parking/cmd_speed", 1);
	steer_pub = nh.advertise<std_msgs::Int16>("/parking/cmd_steer", 1);

	//Suscribirnos al laser
	laser_sub = nh.subscribe("/scan", 1, &autoModelCar::laserCallback, this);

	//Suscribirnos rightUp_side velocidad y angulo
	steer_sub = nh.subscribe("/manual_control/steering", 1, &autoModelCar::steerCallback, this);
	speed_sub = nh.subscribe("/manual_control/speed", 1, &autoModelCar::speedCallback, this);

	//Suscribirnos al desired_angle
	angle_sub = nh.subscribe("/manual_control/desired_angle", 1, &autoModelCar::angleCallback, this);

	// Time variables intialization
	ptime = ros::Time::now();
  	ctime = ros::Time::now();
	ptime = ctime;

	kp = 1.0;

	speed = 0;
	steer = 90;

	state = 0;
}

autoModelCar::~autoModelCar()
{

}

void autoModelCar::steerCallback (const std_msgs::Int16& msg)
{
	steer = msg.data;
}

void autoModelCar::speedCallback (const std_msgs::Int16& msg)
{
	speed = msg.data;
}

void autoModelCar::angleCallback (const std_msgs::Int16 &msg)
{
    dangle = msg.data;
}

void autoModelCar::laserCallback (const sensor_msgs::LaserScan& msg)
{
	laser_msg = msg;
	copy(&msg.ranges[0], &msg.ranges[NUM_DEGREE-1], &laser_distance[0]);
}

int autoModelCar::box_detect(int angle, double leq, double tol){
	double d = laser_distance[angle];
	if ( !isnan( d ) && !isinf( d ) )
		return ( ( d < ( leq + tol) && d > 0.1 ) ? 1 : 0 );	// 1: Encontro caja	0: No encontro
	return 2;	// Es NAN or INF
}

void autoModelCar::Pause(double seconds)
{
    ctime = ros::Time::now();
    ptime = ctime;

    double t = 0.0;
    cout << "/tEn Pausa/t";
    do
    {
        speed = 0;
        steer_16.data = steer;
        speed_16.data = speed;

        steer_pub.publish(steer_16);
        speed_pub.publish(speed_16);
        ctime = ros::Time::now();
        d = ctime - ptime;
        t = d.toSec();

    }while (t <= seconds);

    return;
}

void autoModelCar::parking(){

	ctime = ros::Time::now();
	d = ctime - ptime;
	double t = d.toSec();

	int timef = 0;

	steer = dangle;
	speed = -110;
	int boxret = 0;
	int boxret1 = 0;
	int boxret2 = 0;
	int rightUp_side = 315;
	int right_side = 270;
	double max = 0.5;
	double max2 = 0.70;
	int angulostop;

	double dist = 0.2;

	switch (state) {
		case 0: //no hay caja antes de la primera caja
            speed = -150;
			boxret = box_detect( right_side, max, 0.0);
			if (boxret != 2) {
				//cout << (boxret ? "Box" : "No box") ;
				if (boxret == 1){
					state = 1;
				}
				else
					cout << "Idle" << endl;
			}
			//steer = 5;
			break;
		case 1: // primera caja
            speed = -150;
            boxret = box_detect( right_side, max, 0.0);
			if(boxret != 2){
				if(boxret == 0)
					state = 2;
				else
					cout << "First obstacle detected" << endl;
			}
			//steer = 90;
			break;
		case 2: // no hay caja despues de la primera caja
            speed = -130;			// -110
			boxret = box_detect( right_side, max, 0.0);
			//if (boxret != 2) {
				//cout << (boxret ? "Box" : "No box") ;
				if (boxret == 1){
					state = 3;
					//steer = 90;
				}
				else
					cout << "Detecting free space" << endl;
			//}
			//steer = 10;
			break;
		case 3: //emparejamiento
			//angulostop =  ( 270+(int)(atan2(dist, laser_distance[right_side])) );
			//boxret = box_detect( 225, max, 0.0);}

			// 243 servia en el laboratorio


			speed = -100;
			boxret = box_detect( 243, 0.8, 0.0); // 250 : 245
			boxret1 = box_detect(275, 0.8, 0.0);
			if (boxret !=2) {
				if ( boxret == 1 && boxret1 == 1){
					//if (getchar())
						state = 4;
					speed = 0;
					//steer = 90;
				}
				cout << "Pairing" << endl;
				speed = -110;
			}
			break;
		case 4: //Cambio de llantas antes de retroceder
			steer = 175;
			speed = 0;
			state = 5;
			//Pause(5); 	//ros::Duration(5).sleep();
			break;
		case 5: // checa el momento exacto para detenerse en la vuelta y girar las llantas
			boxret = box_detect( right_side + 10, 0.25, 0.0); // right_side + 10
			boxret1 = box_detect( right_side + 10, 0.35, 0.0); // right_side + 10
			speed = 120;
			steer = 175;
			if (boxret != 2) {
				if ( boxret == 0 && boxret1 == 0){
					state = 6;
				}
				else{
					cout << "Reverse turn" << endl;
				}
			}

			break;
		case 6: //Cambia las llantas
			steer = 5;
			speed = 0;
			state = 7;
			Pause(0.3); 	//ros::Duration(0.3).sleep();
			//getchar();
			//speed = 0;
			//Pause(5);		// ros::Duration(5).sleep();
			break;
		case 7: // estaciona
			speed = 130;
			steer = 5;
			boxret = box_detect( 180, 0.25, 0.0);
			boxret1 = box_detect(160, 0.25, 0.0);
			boxret2 = box_detect(200, 0.25, 0.0);
			if (boxret != 2){
				if (boxret == 1 || boxret1 == 1 || boxret2 == 1){
					state = 8;
				}
				else
					cout << "Checking rear obstacle" << endl;
			}
			//speed = 85;
			break;
		case 8: //cambia las llantas hacia el frente
			steer = 130; //90
			speed = 0;
			state = 9;
			Pause(0.3);		//ros::Duration(0.3).sleep();
			break;
		case 9: // centra el coche
			steer = 130;
			speed = -110;
			boxret = box_detect(180, 0.38, 0.0);
			//boxret1 = box_detect(0, 0.43, 0.0);
			if (boxret != 2) {
				if (boxret == 0) {
				//if (!(laser_distance[0] > laser_distance[180])) {
					state = 10;
				}
			}
			break;
		default:
			steer = 175;
			speed = 0;
			//boxret = box_detect(
			//steer = 90;
			break;
	}

	if ( steer > 180)
        steer = 90;

	//printf("\tS[%d]:L[%d] = %lf\tL[%d] = %lf\t", state, rightUp_side, laser_distance[rightUp_side], right_side, laser_distance[right_side]);
	//printf("T:%lf\tSp:%d\tSt:%d\n", t, speed, steer);

	steer_16.data = steer;
	speed_16.data = speed;

	steer_pub.publish(steer_16);
	speed_pub.publish(speed_16);
}

/*
float autoModelCar::promDist(int angulo, int num)
{

}
*/


int main(int argc, char** argv)
{
	ros::init(argc, argv, "parking");
	autoModelCar c;
	ros::Rate loop_rate(40);
	while(ros::ok())
	{
		c.parking();
		ros::spinOnce();
		loop_rate.sleep();
	}
  return 0;
}

