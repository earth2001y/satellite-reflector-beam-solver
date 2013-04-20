#include <iostream>
#include <cppunit/extensions/HelperMacros.h>
#include <cmath>

#include "../src/satelite_reflector.hpp"
#include "../src/mathematic_utils.hpp"

class SateliteReflectorTest : public CPPUNIT_NS::TestFixture {
  CPPUNIT_TEST_SUITE( SateliteReflectorTest );
  CPPUNIT_TEST( test_find_impact_point );
  CPPUNIT_TEST_SUITE_END();

protected:

public:
  void setUp();
  void tearDown();

protected:
  void test_find_impact_point();

};

CPPUNIT_TEST_SUITE_REGISTRATION( SateliteReflectorTest );

// 各テスト・ケースの実行直前に呼ばれる
void SateliteReflectorTest::setUp() {
}

// 各テスト・ケースの実行直後に呼ばれる
void SateliteReflectorTest::tearDown() {
}

void SateliteReflectorTest::test_find_impact_point() {
  Eigen::Vector3d P0( 30., 40.,  0.);
  Eigen::Vector3d P1(  0., 80.,  0.);
  Eigen::Vector3d  N(  0., 10.,  0.);
  Eigen::Vector3d  O(  0.,  0.,  0.);
  Eigen::Vector3d  R( 50., 50., 50.);

  Vector3dSet X = find_impact_point(P0,P1,N,O,R);
  CPPUNIT_ASSERT(X.size() > 0);

  Eigen::Vector3d P2 = X[0];
  CPPUNIT_ASSERT_DOUBLES_EQUAL(P2[0], -30., 1.e-10);
  CPPUNIT_ASSERT_DOUBLES_EQUAL(P2[1],  40., 1.e-10);
  CPPUNIT_ASSERT_DOUBLES_EQUAL(P2[2],   0., 1.e-10);
}
