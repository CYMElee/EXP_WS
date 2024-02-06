#include "constant.h"

UAV::UAV()
{
    mass = 0.865; //kg
    distance = 0.75; //m

}

Gripper::Gripper()
{
    mass = 0.2125; //kg
    ixyz[0] = 1; //x
    ixyz[1] = 1; //y
    ixyz[2] = 1; //z

}