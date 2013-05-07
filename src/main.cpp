#include <cstdlib>
#include <ctime>
#include <string>
#include <coodinate_system.hpp>
#include "satellite_reflector.hpp"
#include <iostream>

int main(int argc, char* argv[])
{
  geodetic P0,P1;
  P0.latitude  = atof(argv[1]) * M_PI / 180.; // 緯度(北緯) [radian]
  P0.longitude = atof(argv[2]) * M_PI / 180.; // 経度(東経) [radian]
  P0.altitude  = atof(argv[3]) / 1.e3;        // 標高 [km]

  std::string tle_str = std::string(argv[4])  // TLE 1行目
                      + std::string(argv[5]); // TLE 2行目

  // 時刻
  time_t t;
  time(&t);

  // 計算実行
  bool f = find_impact(P0,&P1,tle_str,&t);

  // 結果
  std::cout << (f?'T':'F') << " "; // 着弾があればT、なければF
  if (f) {
    std::cout << P1.latitude  * 180. / M_PI << " " // 緯度(北緯) [degree]
              << P1.longitude * 180. / M_PI << " " // 経度(東経) [degree]
              << P1.altitude  * 1.e3;              // 標高 [m]
  }
  std::cout << std::endl;

  return 0;
}

