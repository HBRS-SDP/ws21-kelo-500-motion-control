#include "PseudoInverse.c"
#include "TorqueTransmission.c"
#include "JacobianMatrix.c"
#include "OptimisationUpdate.c"
#include <stdio.h>

void main()
{
    // 1. initialise arrays and constants
    double radius = 0.1;
    double castor_offset = 0.2;
    double half_wheel_distance = 0.1;
    double motor_const = 3.5714;                                                                    //(A/Nm)
    double wheel_coordinates[8] = {0.175, 0.1605, -0.175, 0.1605, -0.175, -0.1605, 0.175, -0.1605}; //x1,y1,x2,y2,..,y4
    double pivot_angles[4] = {1.0, 0.0, 0.0, 0.0};
    double wheel_torques[8];
    double pivot_forces[8];
    const unsigned int N = 3;
    const unsigned int M = 8;


}