#ifndef CONSTANTS_H
#define CONSTANTS_H

class World
{
    private:
        float g;
    public:
        World();


};



class UAV
{
    private:
        float mass;
        float distance;
    public:

        UAV();

};

class Gripper
{
    private:
        float mass;
        float ixyz[3];
        float damping;
        float Lw;

    public:
         Gripper();


};

#endif