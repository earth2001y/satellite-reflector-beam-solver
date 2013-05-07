/******************************************************************************
 * 反射衛星砲の実計算関数群
 *****************************************************************************/

#ifndef __SATELLITE_REFLECTOR_HPP__
#define __SATELLITE_REFLECTOR_HPP__

#include <vector>
#include <Eigen/Core>
#include "mathematic_utils.hpp"
#include "coodinate_system.hpp"
Vector3dSet find_impact_point(
    const Eigen::Vector3d& P0,
    const Eigen::Vector3d& P1, const Eigen::Vector3d& N,
    const Eigen::Vector3d& O,  const Eigen::Vector3d& R);

bool find_impact(const geodetic& P, geodetic* X,
                 const std::string& tle_str, const time_t* t);

#endif // __SATELLITE_REFLETCTOR_HPP__

