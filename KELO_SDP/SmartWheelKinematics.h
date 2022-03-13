/**
 * @file SmartWheelKinematics.h
 * @author Kavya Shankar (kavya.shankar@smail.inf.h-brs.de)
 * @brief the mapping from pivot forces to wheel torques is formulated
 * @date 2022-03-12
 * 
 */

#ifndef TORQUE_TRANSMISSION_H
#define TORQUE_TRANSMISSION_H

/**
 * @brief for a given pair of wheels, torques are calculated to generate corresponding force
 * 
 * @param pivot_forces forces at individual wheel units
 * @param wheel_torques 1-D array of shape (8,) which is a placeholder for calculated wheel torques for each wheel unit
 * @param radius radius of all wheel units
 * @param castor_offset castor-offset of a pair of wheels from pivot
 * @param half_wheel_distance half of the distance between two wheel units in a pair of wheels
 */

void map_pivot_force_to_wheel_torque(const double *pivot_forces,
                                     double *wheel_torques,
                                     const double radius,
                                     const double castor_offset,
                                     const double half_wheel_distance);
/**
 * @brief pivot forces for every pair of wheels are provide as input to "map_pivot_force_to_wheel_torque" to calculate corresponding torque values
 * 
 * @param pivot_forces forces at individual wheel units
 * @param wheel_torques 1-D array of shape (8,) which is a placeholder for calculated wheel torques for each wheel unit
 * @param radius radius of wheel units
 * @param castor_offset castor-offset of a pair of wheels from pivot
 * @param half_wheel_distance half of the distance between two wheel units in a pair of wheels
 */

void map_pivot_forces_to_wheel_torques(const double *pivot_forces,
                                       double *wheel_torques,
                                       const double radius,
                                       const double castor_offset,
                                       const double half_wheel_distance);

#endif