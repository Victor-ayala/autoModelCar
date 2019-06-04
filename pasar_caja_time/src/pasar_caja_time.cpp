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

#include "pasar_caja_time.h"
using namespace std;
//Vector para guardar laser.ranges[]
//double distancia[4];

//Up, right, down, left angles
const int U_A = 0;
const int R_A = 270;
const int D_A = 180;
const int L_A = 90;

//Velocidades: Adelante, detenerse, reversa
const int A_SPEED = 200;
const int D_SPEED = 0;
const int R_SPEED = -100;

//Steer: Derecha, izquierda
const int D_STEER = 175;
const int I_STEER = 5;
const int DELANTE_ST = 90;


//, it_(nh)

autoModelCar::autoModelCar():nh("~")
{
	ROS_INFO("Init Pasar Caja con tiempo");

	//Publicar
	speed_pub = nh.advertise<std_msgs::Int16>("/manual_control/speed", 1);
	steer_pub = nh.advertise<std_msgs::Int16>("/manual_control/steering", 1);



	obst_pub = nh.advertise<std_msgs::Int16>("/manual_control/obstacle", 1);

	//Suscribirnos al laser
	laser_sub = nh.subscribe("/scan", 1, &autoModelCar::laserCallback, this);

	//Suscribirnos a velocidad y angulo
	steer_sub = nh.subscribe("/manual_control/steering", 1, &autoModelCar::steerCallback, this);
	speed_sub = nh.subscribe("/manual_control/speed", 1, &autoModelCar::speedCallback, this);

	//Suscribirnos al desired_angle
	angle_sub = nh.subscribe("/manual_control/desired_angle", 1, &autoModelCar::angleCallback, this);
	vel_sub = nh.subscribe("/manual_control/desired_speed", 1, &autoModelCar::velCallback, this);

	// Time variables intialization
	ptime = ros::Time::now();
  	ctime = ros::Time::now();
	ptime = ctime;

	speed = 0;
	steer = 90;
	obstacle = 0;

	near = false;

	//Inicializar distancias
	count_d = 0;
	distanciaT[0] = 0;
	distanciaT[1] = 1;

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

void autoModelCar::velCallback (const std_msgs::Int16 &msg)
{
    dvel = msg.data;
}

void autoModelCar::laserCallback (const sensor_msgs::LaserScan& msg)
{
	laser_msg = msg;
	copy(&msg.ranges[0], &msg.ranges[NUM_DEGREE-1], &laser_distance[0]);
}




void autoModelCar::lap()
{
	speed = -120;
	steer = 90;
	obstacle = 0;

	rebasarCaja();



	//Publicar valores
	steer_16.data = steer;
	speed_16.data = speed;
	obstacle_16.data = obstacle;

	steer_pub.publish(steer_16);
	speed_pub.publish(speed_16);
	obst_pub.publish(obstacle_16);
}


//MAIN!!!!
//---------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "MySecondLap");
	autoModelCar c;
	ros::Rate loop_rate(50);
	while(ros::ok())
	{
		// Here you put the code
		c.lap();
		ros::spinOnce();
		loop_rate.sleep();
	}
  return 0;
}

//Funcion que redondea la cantidad al numero de decimales especificados
float redondeo(float cantidad, size_t decimales)
{
	if (cantidad == -1)
	{
		return -1;
	}
	else if(!isnan(cantidad) && !isinf(cantidad))
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
		return -1;
	}
}

/*0	frente
90	izquierda
180	atras
270	derecha*/
float autoModelCar::laserDegree (float degree)
{
	float distancia =  laser_distance[int(degree)];
	if(!isnan(distancia) && !isinf(distancia))
		return distancia;
	else
		return -1;
}

float gradosStop(float dist)
{
	const float pendiente = -9.25925;
	float grados;
	grados = pendiente*dist + 45;
	return grados;
}

void autoModelCar::rebasarCaja()
{
	ctime = ros::Time::now();
	d = ctime - ptime;
	t = d.toSec();

	int angulo = 0;
	float distancia = -1;

	const float maxVel = -200;

	//Checar que dangle sea valido
	if (dangle < 0)
		dangle = 90;
	if (dangle > 180)
		dangle = 90;

	cout << "STATE: " << state << "\tTIME: " << t << " ";

	switch (state)
	{
		//Llegar a la distancia que se quiere
		case 0:
			//Valores de default
			speed = -200;
			steer = dangle;
			angulo = 5;
			obstacle = 0;

			//Conseguir el valor de la distancia usando el promedio de tres valores
			distancia = distProm(0,1);
			//cin >> distancia;

			//Revisar que dvel sea un numero valido (dvel = desired velocity)
			if (dvel>-260 && dvel < 0)
				speed = dvel;
			else
				speed = -180;

			/*Cuando se empieza a acercar a un obstaculo, bajar
			la velocidad*/
			if (distancia < 1.5 && distancia > 0.15)
				speed = -180;

			//Si esta mas lejos que 0.9, se pone a near como falso
			if (distancia > 0.93 && distancia < 1.5)
			{
				near = false;
			}

			//No moverse si near es verdadero
			if (near == true)
			{
				speed = 0;
				steer = 90;
			}

			/* Si estamos dentro del rango de la distancia, near es verdadero
			y se renicia el contador de tiempo */
			if (distancia < 0.7 && distancia > 0.15 && near == false)
			{
				near = true;
				ctime = ros::Time::now();
				ptime = ctime;
				break;
			}

			/* Si el tiempo supera 5 segundos y near es verdadero, pasar
			al siguiente estado */
			if (t > 5.0 && near == true)
			{
				ctime = ros::Time::now();
				ptime = ctime;
				state++;
			}

			cout << "\tSPEED: " << dvel << "\tNEAR: " << near;
		break;

		/* Dar vuelta a la izquierda y registrar el tiempo hasrta que se
		encuentre la caja */
		case 1:
			speed = maxVel;
			steer = I_STEER;
			angulo = 290;
			obstacle = -1;

			distancia = distProm(angulo,1);

			if (distancia < 0.5 && distancia > 0.15)
			{
				tiempoV = t;
				if (tiempoV > 2)
					tiempoV = 2;

				ctime = ros::Time::now();
				ptime = ctime;
				cout << "\t" << tiempoV << "\t";
				state ++;
			}
		break;

		//Dar vuelta a la derecha la cantidad de tiempo conseguida en el caso1
		case 2:
			speed = maxVel;
			steer = D_STEER;
			obstacle = -1;

			if (t >= tiempoV)
			{
				ctime = ros::Time::now();
				ptime = ctime;
				state ++;
			}
		break;

		//Seguir en linea recta por un momento
		case 3:
			speed = maxVel;
			steer = 90;
			obstacle = -1;

			if (t >= 0.2)
			{
				ctime = ros::Time::now();
				ptime = ctime;
				state ++;
			}
		break;

		/* Dar vuelta a la derecha por el tiempo consguido en el caso1 y
		regresar al estado inicial */
		case 4:
			speed = maxVel;
			steer = D_STEER;
			obstacle = -1;

			if (t >= tiempoV)
			{
				ctime = ros::Time::now();
				ptime = ctime;
				state = 0;
			}
		break;

		default:
			speed = 0;
			steer = 90;
			obstacle = -1;
	}

	cout << endl;

}


float autoModelCar::distProm(int grado, int num)
{
	int div = 0;
	const float max = 1;
	const float min = 0.14;
	float suma = 0;

	int grado_n = 0;


	for (int i = -num; i<num+1; i++)
	{
		grado_n = grado+i;

		if (grado_n > 359)
			grado_n = -1+i;

		if (grado_n < 0)
			grado_n = 360+i;

		if(laserDegree(grado_n) > min && laserDegree(grado_n) < max)
		{
			suma += laserDegree(grado_n);
			div ++ ;
		}
	}
	if (div > 0)
		suma /= div;
	else
		return -1;

	//cout << div << "\t";
	return suma;
}

//Calcular distancia en el instante
void autoModelCar::calcDI(int angulo)
{
	//Distancias
	if (count_d == 0)
	{
		distanciaT[2] = distanciaT[0];
		distanciaT[1] = distProm(angulo,2);

		count_d = 1;
	}
	else if (count_d == 1)
	{
		distanciaT[0] = distProm(angulo,2);
		distanciaT[2] = distanciaT[0];
		count_d = 0;
	}
}
