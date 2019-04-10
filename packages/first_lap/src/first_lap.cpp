/*
 * First lap autoModelCar
 *
 * Author:	Victor Ayala Alfaro
 * 			Vision, Robotics and Artificial Intelligence Laboratory
 * 			LaViRIA
 *
 * 03/2018
 *
 * */

#include "first_lap.h"
using namespace std;
//Vector para guardar laser.ranges[]
//double distancia[4];


autoModelCar::autoModelCar():nh("~"), it_(nh)
{
	ROS_INFO("Init My First Lap");

	//Publicar velocidad del carrito (para simulacion)
	twist_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

	//Suscribirnos al laser
	laser_sub = nh.subscribe("/scan", 1, &autoModelCar::laserCallback, this);

	steer_sub = ("/manual_control/steer", 1, &autoModelCar::steerCallback, this);
	angle_sub = nh.subscribe("/manual_control/desired_angle", 1, &autoModelCar::angleCallback, this)

	// Time variables intialization
	ptime = ros::Time::now();
  	ctime = ros::Time::now();
	ptime = ctime;
	state = 0;
}

autoModelCar::~autoModelCar()
{

}

int counter = 0;
float distanciaI = 0;
float distanciaF = 0;
float gradoS = 45;

const float Vfor = 0.2;
const float Vbac = -0.2;
const float Vstop = 0;

const float Srig = -0.4;
const float Slef = 0.4;
const float Sstop = 0;


void autoModelCar::lap()
{

	ctime = ros::Time::now();
	d = ctime - ptime;
	double t = d.toSec();

	//Steer = velocidad angular, Speed = velocidad linear
	double steer = 0, speed = 0;

	//Estacionar (simulacion)
	switch (counter)
	{
		/*Avanzar y registrar la distancia hasta llegar al final de
		la primera caja*/
		case 0:
			distanciaF = laserDegree(90);
			speed = Vfor;
			steer = 0;
			if (distanciaI == 0)
			{
				distanciaI = laserDegree(90);
			}

			if (redondeo(laserDegree(90),2) > 6)
			{
				gradoS = gradosStop(distanciaI);
				counter ++;
				ptime = ctime;
				break;
			}
			if (redondeo(distanciaF, 0) > redondeo(distanciaI, 0) )
			{
				cout << "Izquierda! \t";
				steer = 0.2;
				speed = 0.1;
			}
			else if (redondeo(distanciaF, 0) < redondeo(distanciaI, 0) )
			{
				cout << "Derecha!\t";
				steer = -0.2;
				speed = 0.1;
			}

			/*
			if(redondeo(laserDegree(90),2) < 6)
			{
				distanciaI = laserDegree(90);
			}
			else
			{
				gradoS = gradosStop(distanciaI);
				counter ++;
				ptime = ctime;
			}*/
			break;
		/*Avanzar hasta detectar la segunda caja*/
		case 1:
			steer = 0;
			speed = Vfor;
			if (redondeo(laserDegree(90),2) < 6 && t > 2.0)
			{
				counter ++;
				ptime = ctime;
			}
			break;

		case 2:
			steer = 0;
			distanciaF = laserDegree(90);
			speed = 0.1;
			gradoS = gradosStop(redondeo(distanciaI, 2));

			if (redondeo(distanciaF, 0) > redondeo(distanciaI, 0)  )
			{
				cout << "Izquierda!\t";
				steer = 0.2;
				speed = 0.1;
				break;
			}
			else if (redondeo(distanciaF, 0) < redondeo(distanciaI, 0)  )
			{
				cout << "Derecha!\t";
				steer = -0.2;
				speed = 0.1;
				break;
			}

			if (redondeo(laserDegree(90 - gradoS) ,1) == redondeo(laserDegree(90 + gradoS) ,1))
			{
				cout << endl << redondeo(laserDegree(90 - gradoS) ,1)
				<<"\t" << redondeo(laserDegree(90 + gradoS) ,1) << endl;
				speed = Vstop;
				counter++;
				ptime = ctime;
				break;
			}
			else if (redondeo(laserDegree(90),2) < 6)
				speed = 0.1;
			else if(redondeo(laserDegree(90),2) > 6 && t > 1)
				speed = Vstop;
				break;
			break;

		case 3:
			if (t < 6.5)
			{
				speed = Vbac;
				steer = Srig;
			}
			else
			{
				speed = Vstop;
				steer = Sstop;
				ptime = ctime;
				counter++;
			}
			break;
		case 4:
			if (t<3.0)
			{
				speed = Vbac;
				steer = Slef;
			}
			else
			{
				speed = Vstop;
				steer = Sstop;
				ptime = ctime;
				counter++;
			}
			break;
		case 5:
			if(t<3.0)
			{
				speed = Vfor;
				steer = Slef;
			}
			else
			{
				speed = Vstop;
				steer = Sstop;
				ptime = ctime;
				counter++;
			}
			break;
		//Ponerse en medio de las dos cajas
		case 6:
			if (redondeo(laserDegree(0),3) > redondeo(laserDegree(180),3))
			{
				cout << "Adelante!" << "\t\t" <<redondeo(laserDegree(0),3) <<"\t"<< redondeo(laserDegree(180),3) <<endl;
				speed = 0.1;
			}
			else if (redondeo(laserDegree(0),3) < redondeo(laserDegree(180),3))
			{
				cout << "Reversa!" << "\t\t" <<redondeo(laserDegree(0),3) <<"\t"<< redondeo(laserDegree(180),3) <<endl;
				speed = -0.1;
			}
			if (redondeo(laserDegree(0),2) == redondeo(laserDegree(180),2))
			{
				cout << "Listo!" << "\t\t" <<redondeo(laserDegree(0),3) <<"\t"<< redondeo(laserDegree(180),3) <<endl;
				counter ++;
				speed = 0;
			}
			break;
		default:
			speed = 0;
			break;
	}
	//Imprimir informacion (simulacion)
	if (counter == 6)
		cout <<redondeo(laserDegree(0),3) <<"\t"<< redondeo(laserDegree(180),3) <<endl;
	else if (counter < 6 )
	{
		cout << "C: " << counter
		//<< "\tL90: " << redondeo(laserDegree(90),1)
		//<< "\tGs: " << gradosStop(distancia)
		//<< "\tGG: " << 90-(gradoS/2)
		//<< "\tLi: " << redondeo(laserDegree(90 - gradoS) ,1)
		//<< "\tLs: " << redondeo(laserDegree(90 + gradoS) ,1)
		<< "\t\t\tT: " << t
		<< "\tDi: " << distanciaI
		<< "\tDf: " << distanciaF
		<< "\tDf-Di: "<< distanciaF-distanciaI
		//<< "\tGG: " << gradoS
		<<endl;
	}
	//Publicar informacion (simulacion)
	twist_msg.angular.z = steer;
	twist_msg.linear.x = speed;

	twist_pub.publish(twist_msg);
}

void autoModelCar::laserCallback (const sensor_msgs::LaserScan& laser)
{
	laser_msg = laser;
	for (size_t i = 0; i<NUM_DEGREE; i++)
		laser_distance[i] = laser.ranges[i];
}


/*0	frente
90	izquierda
180	atras
270	derecha*/
float autoModelCar::laserDegree (float degree)
{
	if (degree > NUM_DEGREE)
		return laser_distance[int((degree-360)*2)];
	else if (degree < 0)
		return laser_distance[int((degree+360)*2)];
	else
		return laser_distance[int(degree*2)];
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "MyFirstLap");
  autoModelCar c;
	while(ros::ok())
	{
		// Here you put the code
		c.lap();
		ros::spinOnce();
	}
  return 0;
}

//Funcion que redondea la cantidad al numero de decimales especificados
float redondeo(float cantidad, size_t decimales)
{
	if(!isnan(cantidad) && !isinf(cantidad))
	{
		//Se recorre el punto decimal del flotante el numero de veces que se quieren de decimales para redondear
		decimales = pow(float(10) , float(decimales));
		cantidad = cantidad * decimales;

		//roundf es una funcion de cmath que redondea la cantidad en punto flotante
		cantidad = roundf(cantidad);
		//Se regresa el punto decimal a su lugar original
		cantidad /= decimales;

		return cantidad;
	}
	else
	{
		return 1000000000000;
	}
}


const float pendiente = -9.25925;
float gradosStop(float dist)
{
	float grados;
	grados = pendiente*dist + 45;
	return grados;
}
