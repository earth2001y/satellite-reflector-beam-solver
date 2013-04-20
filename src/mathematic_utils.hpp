/******************************************************************************
 * 反射衛星砲: 数学モデルユーティリティ関数
 *****************************************************************************/

#ifndef __MATHEMATIC_UTILS_HPP__
#define __MATHEMATIC_UTILS_HPP__

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

typedef std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > Vector3dSet;

// 点P1,P2の間の距離の2乗を求める
double distance2(const Eigen::Vector3d& P1, const Eigen::Vector3d& P2);

// ベクトルX を正規化したベクトルを求める
Eigen::Vector3d normalize(const Eigen::Vector3d& X);

// ベクトルX が ベクトルa を軸として p[rad]回転したときのベクトルを求める
Eigen::Vector3d rotate(const Eigen::Vector3d& X, const Eigen::Vector3d& a, const double p);

// ベクトルX が 法線ベクトルn の面に入射したときの反射ベクトルを求める
Eigen::Vector3d reflect(const Eigen::Vector3d& X, const Eigen::Vector3d& n);

// 点の集合P={p1,p2,...} のうち、点Oに最も近い点の集合(⊆P)を求める
Vector3dSet neighoring(const Eigen::Vector3d& O, const Vector3dSet& P);

// 直線L(P,v) が楕円体E(O,R)の表面を通過,接触する座標を求める
// ただし実数解のみ
Vector3dSet intersection(const Eigen::Vector3d& P, const Eigen::Vector3d& v,
                       const Eigen::Vector3d& O, const Eigen::Vector3d& R);

// 点P が楕円体E(O,R)の中にあるかどうか調べる
bool inner_elipsoid(const Eigen::Vector3d& P, const Eigen::Vector3d& O, const Eigen::Vector3d& R);

#endif // __MATHEMATIC_UTILS_HPP__

