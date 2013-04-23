#include <iostream>
#include <cppunit/extensions/HelperMacros.h>
#include <cmath>
#include <string>
#include <ctime>

#include "../src/satellite_reflector.hpp"
#include "../src/mathematic_utils.hpp"
#include "coodinate_system.hpp"

class SatelliteReflectorTest : public CPPUNIT_NS::TestFixture {
  CPPUNIT_TEST_SUITE( SatelliteReflectorTest );
  CPPUNIT_TEST( test_find_impact_point );
  CPPUNIT_TEST( test_find_impact );
  CPPUNIT_TEST_SUITE_END();

protected:

public:
  void setUp();
  void tearDown();

protected:
  void test_find_impact_point();
  void test_find_impact();

};

CPPUNIT_TEST_SUITE_REGISTRATION( SatelliteReflectorTest );

// 各テスト・ケースの実行直前に呼ばれる
void SatelliteReflectorTest::setUp() {
}

// 各テスト・ケースの実行直後に呼ばれる
void SatelliteReflectorTest::tearDown() {
}

void SatelliteReflectorTest::test_find_impact_point() {
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

void SatelliteReflectorTest::test_find_impact() {
#include <iostream>
using namespace std;
  // HST
  std::string tle;
  tle = std::string("1 20580U 90037B   13107.72940891  .00001584  00000-0  10118-3 0  2193")
      + std::string("2 20580  28.4702 309.1211 0003456  63.1635  29.3970 15.03482263 60582");

  // Tokyo
  ::polar P0;
  P0.latitude  = ( 35. + (41./60.));
  P0.longitude = (139. + (46./60.));
  P0.altitude  = 0.01;

  // now
  time_t t;
  time(&t);

  // find impact
  ::polar P1;
  bool f = find_impact(P0,&P1,tle,&t);

  cout << endl;
  if (f) {
    cout << "N: " << P1.latitude  << endl
         << "E: " << P1.longitude << endl
         << "H: " << P1.altitude  << endl;
  } else {
    cout << " no impact " << endl;
  }
}
