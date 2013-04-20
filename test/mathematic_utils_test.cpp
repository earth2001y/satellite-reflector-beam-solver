#include <iostream>
#include <cppunit/extensions/HelperMacros.h>
#include <cmath>

#include "../src/mathematic_utils.hpp"

class MathematicUtilsTest : public CPPUNIT_NS::TestFixture {
  CPPUNIT_TEST_SUITE( MathematicUtilsTest );
  CPPUNIT_TEST( test_distance2 );
  CPPUNIT_TEST( test_normalize );
  CPPUNIT_TEST( test_normalize );
  CPPUNIT_TEST( test_rotate );
  CPPUNIT_TEST( test_neihoring );
  CPPUNIT_TEST( test_reflect );
  CPPUNIT_TEST( test_intersection );
  CPPUNIT_TEST_SUITE_END();

protected:

public:
  void setUp();
  void tearDown();

protected:
  void test_distance2();
  void test_inner_elipsoid();
  void test_normalize();
  void test_rotate();
  void test_neihoring();
  void test_reflect();
  void test_intersection();

};

CPPUNIT_TEST_SUITE_REGISTRATION( MathematicUtilsTest );

// 各テスト・ケースの実行直前に呼ばれる
void MathematicUtilsTest::setUp() {
}

// 各テスト・ケースの実行直後に呼ばれる
void MathematicUtilsTest::tearDown() {
}

void MathematicUtilsTest::test_distance2() {
  Eigen::Vector3d P1, P2;

  P1 << 1., 0., 0.;
  P2 << 0., 0., 0.;
  CPPUNIT_ASSERT_DOUBLES_EQUAL(distance2(P1,P2), 1., 1.e-12);

  P1 << 0., 1., 0.;
  P2 << 0., 0., 0.;
  CPPUNIT_ASSERT_DOUBLES_EQUAL(distance2(P1,P2), 1., 1.e-12);

  P1 << 0., 0., 1.;
  P2 << 0., 0., 0.;
  CPPUNIT_ASSERT_DOUBLES_EQUAL(distance2(P1,P2), 1., 1.e-12);

  P1 << 1., 1., 1.;
  P2 << 0., 0., 0.;
  CPPUNIT_ASSERT_DOUBLES_EQUAL(distance2(P1,P2), 3., 1.e-12);

  P1 << 0., 0., 0.;
  P2 << 1., 1., 1.;
  CPPUNIT_ASSERT_DOUBLES_EQUAL(distance2(P1,P2), 3., 1.e-12);

}

void MathematicUtilsTest::test_normalize() {
  Eigen::Vector3d P0,P1,P2,P3;
  Eigen::Vector3d N0,N1,N2,N3;

  P0 << 0.,0.,0.;
  P1 << 2.,0.,0.;
  P2 << 0.,2.,0.;
  P3 << 0.,0.,2.;

  N0 << 0.,0.,0.;
  N1 << 1.,0.,0.;
  N2 << 0.,1.,0.;
  N3 << 0.,0.,1.;

  CPPUNIT_ASSERT(normalize(P0) == N0);
  CPPUNIT_ASSERT(normalize(P1) == N1);
  CPPUNIT_ASSERT(normalize(P2) == N2);
  CPPUNIT_ASSERT(normalize(P3) == N3);
}

void MathematicUtilsTest::test_rotate() {
  Eigen::Vector3d P0,P1,P2,P3;
  Eigen::Vector3d Ax,Ay,Az;

  P0 << 0.,0.,0.;
  P1 << 2.,0.,0.;
  P2 << 0.,2.,0.;
  P3 << 0.,0.,2.;

  Ax << 1.,0.,0.;
  Ay << 0.,1.,0.;
  Az << 0.,0.,1.;

  CPPUNIT_ASSERT(rotate(P1,Ax,M_PI) == Eigen::Vector3d(P1));
  CPPUNIT_ASSERT(rotate(P2,Ay,M_PI) == Eigen::Vector3d(P2));
  CPPUNIT_ASSERT(rotate(P3,Az,M_PI) == Eigen::Vector3d(P3));
}

void MathematicUtilsTest::test_neihoring() {
  Eigen::Vector3d O(1.,1.,1.);
  Eigen::Vector3d P1,P2,P3,P4,P5,P6;
  Vector3dSet P,Q;

  P1 << -1., 0., 0.;
  P2 <<  1., 0., 0.;
  P3 <<  0.,-1., 0.;
  P4 <<  0., 1., 0.;
  P5 <<  0., 0.,-1.;
  P6 <<  0., 0., 1.;

  P.push_back(P1);
  P.push_back(P2);
  P.push_back(P3);
  P.push_back(P4);
  P.push_back(P5);
  P.push_back(P6);

  Q = neighoring(O,P);
  CPPUNIT_ASSERT(Q.size() == 3);

}

void MathematicUtilsTest::test_reflect() {
  Eigen::Vector3d X1,X2,X3;
  Eigen::Vector3d N1,N2,N3;

  X1 << 2., 0., 0.;
  X2 << 0.,-4., 0.;
  X3 << 0., 0.,-3.;

  N1 << 1., 0., 0.;
  N2 << 0., 2., 0.;
  N3 << 0., 0., 3.;

  CPPUNIT_ASSERT(reflect(X1,N1) == Eigen::Vector3d(-2.,0.,0.));
  CPPUNIT_ASSERT(reflect(X1,N2) == Eigen::Vector3d( 2.,0.,0.));
  CPPUNIT_ASSERT(reflect(X2,N2) == Eigen::Vector3d( 0.,4.,0.));
  CPPUNIT_ASSERT(reflect(X3,N3) == Eigen::Vector3d( 0.,0.,3.));
}

void MathematicUtilsTest::test_intersection() {

  Eigen::Vector3d P0,P1,P2,P3;
  Eigen::Vector3d v1,v2,v3;
  Eigen::Vector3d O(0., 0., 0.);
  Eigen::Vector3d R(10.,20.,30.);
  Vector3dSet Q1,Q2,Q3;

  P0 << 0.,0.,0.;
  P1 << 10.,0.,0.;
  P2 << 0.,20.,0.;
  P3 << 0.,0.,30.;

  v1 << 1.,0.,0.;
  v2 << 0.,1.,0.;
  v3 << 0.,0.,1.;

  Q1 = intersection(P0,v1,O,R);
  Q2 = intersection(P0,v2,O,R);
  Q3 = intersection(P0,v3,O,R);
  CPPUNIT_ASSERT(Q1.size() == 2);
  CPPUNIT_ASSERT(Q2.size() == 2);
  CPPUNIT_ASSERT(Q3.size() == 2);

  Q1 = intersection(P1,v1,O,R);
  Q2 = intersection(P1,v2,O,R);
  Q3 = intersection(P1,v3,O,R);
  CPPUNIT_ASSERT(Q1.size() == 2);
  CPPUNIT_ASSERT(Q2.size() == 1);
  CPPUNIT_ASSERT(Q3.size() == 1);

  Q1 = intersection(P2,v1,O,R);
  Q2 = intersection(P2,v2,O,R);
  Q3 = intersection(P2,v3,O,R);
  CPPUNIT_ASSERT(Q1.size() == 1);
  CPPUNIT_ASSERT(Q2.size() == 2);
  CPPUNIT_ASSERT(Q3.size() == 1);

  Q1 = intersection(P3,v1,O,R);
  Q2 = intersection(P3,v2,O,R);
  Q3 = intersection(P3,v3,O,R);
  CPPUNIT_ASSERT(Q1.size() == 1);
  CPPUNIT_ASSERT(Q2.size() == 1);
  CPPUNIT_ASSERT(Q3.size() == 2);

  Vector3dSet Q = intersection(
                    Eigen::Vector3d(0.,80.,0.),
                    Eigen::Vector3d(-30.,-40.,0.),
                    Eigen::Vector3d(0.,0.,0.),
                    Eigen::Vector3d(50.,50.,50.));
}

