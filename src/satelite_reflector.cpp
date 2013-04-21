#include <cmath>
#include <string>
#include <coodinate_system.hpp>
#include <tle.hpp>
#include <orbit.hpp>
#include <ctime>
#include <Eigen/Core>
#include <Eigen/Geometry>
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

// 測地座標系で、
//  射点位置(P)
//  使う人工衛星のTLE
//  現在時刻 t
// から、地上の着弾点を得る
bool find_impact(const polar& P, polar* X,
                 const std::string& tle_str, const time_t* t)
{
  // TLE読み込み
  TLE tle;
  std::string tle_str1 = tle_str.substr(0,69);
  std::string tle_str2 = tle_str.substr(69,69);
  tle.set(tle_str1,tle_str2);

  // 地心赤道直交座標系での射点座標
  ::polar Po = P;
  Po.latitude  *= M_PI / 180.; // degree -> radian
  Po.longitude *= M_PI / 180.; // degree -> radian
  rectangular Pr = Po.toEquatorial(t).toRectangular();
  Eigen::Vector3d P0(Pr.X, Pr.Y, Pr.Z);

  // 軌道
  orbit orb;
  orb.setTLE(&tle);
  double since_day = orb.elapsed_day(t);
  double since_min = since_day * 1440.;
  double since_sec = since_day * 86400.;

  double motion = tle.motion; // revolutions per day
  double dayp6  = 1. / (6. * motion); // day for 1/6 revolution.
  double minp6  = dayp6 * 1440.;
  double secp6  = dayp6 * 86400.;

  // 人工衛星の所在地
  double position0[3],position1[3],position2[3];
  double velocity0[3],velocity1[3],velocity2[3];
  orb.sgp(position0,velocity0,since_min - minp6);
  orb.sgp(position1,velocity1,since_min);
  orb.sgp(position2,velocity2,since_min + minp6);

  Eigen::Vector3d p0(position0);
  Eigen::Vector3d p1(position1); // 人工衛星の現在地の位置ベクトル
  Eigen::Vector3d p2(position2);
  Eigen::Vector3d v1(velocity1);

  Eigen::Vector3d n0 = normalize((p1-p0).cross(p2-p1)); // 軌道面の法線ベクトル
  Eigen::Vector3d n1 = n0.cross(normalize(v1));         // 反射面の法線ベクトル

  Eigen::Vector3d O(0.,0.,0.);
  Eigen::Vector3d R(6378.137,6378.137,6356.752); // 地球楕円体
  Vector3dSet Q = find_impact_point(P0,p1,n1,O,R);

  rectangular Xr;
  bool f = (Q.size() != 0);
  if (f) {
    Eigen::Vector3d q = Q[0];
    Xr.X = q[0];
    Xr.Y = q[1];
    Xr.Z = q[2];
  } else {
    Xr.X = 0.0;
    Xr.Y = 0.0;
    Xr.Z = 0.0;
  }

  ::polar x = Xr.toPolar();
  *X = Xr.toPolar().toGeodetic(t); // 測地座標系に変換
  X->latitude  *= 180. / M_PI;     // radian -> degree
  X->longitude *= 180. / M_PI;     // radian -> degree
  if      ( X->latitude  >  180.) X->latitude  -= 360.;
  else if ( X->latitude  < -180.) X->latitude  += 360.;
  if      ( X->longitude >  180.) X->longitude -= 360.;
  else if ( X->longitude < -180.) X->longitude += 360.;

  return f;
}

