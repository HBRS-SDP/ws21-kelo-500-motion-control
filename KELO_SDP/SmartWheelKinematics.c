/**
 * @file SmartWheelKinematics.h
 * @author Kavya Shankar (kavya.shankar@smail.inf.h-brs.de)
 * @brief the mapping from pivot forces to wheel torques is formulated
 * @date 2022-03-12
 * 
 */

#include <stdio.h>
#include "SmartWheelKinematics.h"

#define LEFT_WHEEL 0
#define RIGHT_WHEEL 1
#define X_COMPONENT 0
#define Y_COMPONENT 1

void map_pivot_force_to_wheel_torque(const double *pivot_forces,
                                     double *wheel_torques,
                                     const double radius,
                                     const double castor_offset,
                                     const double half_wheel_distance)
{
    wheel_torques[LEFT_WHEEL] = (radius / 2) * (pivot_forces[Y_COMPONENT] / (castor_offset / half_wheel_distance) + pivot_forces[X_COMPONENT]);
    wheel_torques[RIGHT_WHEEL] = (radius / 2) * (-pivot_forces[Y_COMPONENT] / (castor_offset / half_wheel_distance) + pivot_forces[X_COMPONENT]);
}

void map_pivot_forces_to_wheel_torques(const double *pivot_forces,
                                       double *wheel_torques,
                                       const double radius,
                                       const double castor_offset,
                                       const double half_wheel_distance)
{
    for (int wheel_pair_index = 0; wheel_pair_index < 4; wheel_pair_index++)
    {
        map_pivot_force_to_wheel_torque(&pivot_forces[2 * wheel_pair_index],
                                        &wheel_torques[2 * wheel_pair_index],
                                        radius,
                                        castor_offset,
                                        half_wheel_distance);
    }
}
