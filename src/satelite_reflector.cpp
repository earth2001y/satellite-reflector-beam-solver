#include <cmath>
#include "mathematic_utils.hpp"

using namespace Eigen;

// 地心直交座標系で、
//  射点位置(P0)
//  衛星位置(P1)
//  軌道接平面法線(N)
// から、
//  楕円体(E(O,R)) 面上の着弾点を得る
Vector3dSet find_impact_point(
    const Vector3d& P0, const Vector3d& P1, const Vector3d& N,
    const Vector3d& O, const Vector3d& R)
{
  Vector3d v1 = P1 - P0;                   // 発射ビームの向き
  Vector3d v2 = reflect(v1,N);             // 反射ビームの向き
  Vector3dSet Q = intersection(P1,v2,O,R); // 着弾点候補
  Vector3dSet X = neighoring(P1,Q);        // 着弾点候補のうちP1に近い点

  return X;
}

