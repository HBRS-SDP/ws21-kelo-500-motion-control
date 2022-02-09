#ifndef TORQUE_TRANSMISSION_H
#define TORQUE_TRANSMISSION_H

void map_pivot_force_to_wheel_torque(const double *pivot_forces,
                                     double *wheel_torques,
                                     double radius,
                                     double castor_offset,
                                     double half_wheel_distance);

void map_pivot_forces_to_wheel_torques(const double *pivot_forces,
                                       double *wheel_torques,
                                       double radius,
                                       double castor_offset,
                                       double half_wheel_distance);

#endif