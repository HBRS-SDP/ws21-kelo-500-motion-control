#include <stdio.h>
#include "TorqueTransmission.h"

void map_pivot_force_to_wheel_torque(const double *pivot_forces,
                                     double *wheel_torques,
                                     double radius,
                                     double castor_offset,
                                     double half_wheel_distance)
{
    wheel_torques[0] = (radius/2)*(pivot_forces[0]/(half_wheel_distance/castor_offset) + pivot_forces[1]);
    wheel_torques[1] = (radius/2)*(-pivot_forces[0]/(half_wheel_distance/castor_offset) + pivot_forces[1]);
}

void map_pivot_forces_to_wheel_torques(const double *pivot_forces,
                                       double *wheel_torques,
                                       double radius,
                                       double castor_offset,
                                       double half_wheel_distance)
{
    for (int i = 0; i < 4; i++)
    {
        map_pivot_force_to_wheel_torque(&pivot_forces[2 * i],
                                        &wheel_torques[2 * i],
                                        radius,
                                        castor_offset,
                                        half_wheel_distance);
    }
}