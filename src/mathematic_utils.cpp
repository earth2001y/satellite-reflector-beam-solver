#include <cmath>
#include "mathematic_utils.hpp"

using namespace Eigen;

double distance2(const Vector3d& P1, const Vector3d& P2)
{
  Vector3d dP = P2 - P1;
  return dP.dot(dP);
}

Vector3d normalize(const Vector3d& X)
{
  Vector3d N = X;
  if (N == Vector3d(0.,0.,0.)) return N;
  N.normalize();
  return N;
}

Vector3d rotate(const Vector3d& X, const Vector3d& a, const double p)
{
  Vector3d n = normalize(a);
  return X * cos(p) + n * (1.-cos(p)) * X.dot(n) + n.cross(X*sin(p));
}

bool inner_elipsoid(const Vector3d& P, const Vector3d& O, const Vector3d& R)
{
  Vector3d Ps = (P - O).array() / R.array();
  return (Ps.array() * Ps.array()).sum() <= 1.;
}

Vector3dSet neighoring(const Vector3d& O, const Vector3dSet& P)
{
  Vector3dSet Q;
  double d;

  if ( P.size() == 0 ) return Q;

  Q.push_back(P[0]);
  d = distance2(P[0],O);
  for (size_t p = 1; p < P.size(); p++ ) {
    double d2 = distance2(P[p],O);
    if (d2 == d) {
      Q.push_back(P[p]);
    } else if (d2 < d) {
      Q.clear();
      Q.push_back(P[p]);
      d = d2;
    }
  }

  return Q;
}

Vector3d reflect(const Vector3d& X, const Vector3d& n)
{
  Vector3d Y;
  if (X.dot(n) == 0.) {
    // 特異点: Xとnが直交する
    Y = X;
  } else if ((X.cross(n).array() == 0.).all()) {
    // 特異点: Xとnが平行している
    Y = -X;
  } else {
    double lx = sqrt(X.dot(X));
    double ln = sqrt(n.dot(n));
    double p  = acos(X.dot(n)/(lx*ln));
    Vector3d a  = X.cross(n);
    Y = -rotate(X,a,2.*p);
  }

  return Y;
}

Vector3dSet intersection(const Vector3d& P, const Vector3d& v,
                         const Vector3d& O, const Vector3d& R)
{
  Vector3dSet Q;
  Vector3d P0 = P - O;

  // Q = P + v.t として
  // (Qx/a)**2 + (Qy/b)**2 + (Qz/c)**2 = 1
  // となる方程式 A.t**2 + B.t + C = 0 を t について解く
  double A = ((v.array() / R.array()) * (v.array() / R.array())).sum();
  double B = ( P0.array() * v.array() * 2. / (R.array() * R.array())).sum();
  double C = ((P.array() / R.array()) * (P0.array() / R.array())).sum() - 1.;

  // 解の判定
  double D = B*B - 4.*A*C;
  if ( D >= 0. ) {
    // 実数解になる
    double t1 = (-B + sqrt(D)) / (2. * A);
    double t2 = (-B - sqrt(D)) / (2. * A);

    // tをLの方程式に当てて交点Qとする
    Vector3d Q1 = P + v*t1;
    Vector3d Q2 = P + v*t2;

    if ( D == 0. ) {
      // 解はひとつ
      Q.push_back(Q1);
    } else {
      // 解は2つ
      Q.push_back(Q1);
      Q.push_back(Q2);
    }
  }

  return Q;
}

