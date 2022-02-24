#ifndef TORQUE_TRANSMISSION_H
#define TORQUE_TRANSMISSION_H

void map_pivot_force_to_wheel_torque(const double *pivot_forces,
                                     double *wheel_torques,
                                     const double radius,
                                     const double castor_offset,
                                     const double half_wheel_distance);

void map_pivot_forces_to_wheel_torques(const double *pivot_forces,
                                       double *wheel_torques,
                                       const double radius,
                                       const double castor_offset,
                                       const double half_wheel_distance);

#endif