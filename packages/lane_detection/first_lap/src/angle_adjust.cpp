#include "first_lap.h"

void autoModelCar::steerCallback (const std_msgs::Int16 &steer)
{
    steer_16 = steer;
}



void autoModelCar::angleCallback (const std_msgs::Int16 &msg)
{
    const float k = 0;
    Dangle_16 = k*(msg-steer);
}
