#ifndef CONSTANTS_H
#define CONSTANTS_H

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

    public:
         Gripper();


};

#endif