#include "constant.h"

UAV::UAV()
{
    mass = 0.865; //kg
    distance = 0.75; //m
}

Gripper::Gripper()
{
    Lw = 0.75; // meter
    mass = 0.2125; //kg
    ixyz[0] = 1; //x
    ixyz[1] = 1; //y
    ixyz[2] = 1; //z

}


World::World()
{
        g = 9.81;   //   m/s^2


}