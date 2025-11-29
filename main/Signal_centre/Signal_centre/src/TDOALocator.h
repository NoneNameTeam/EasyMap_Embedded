#ifndef TDOA_LOCATOR_H
#define TDOA_LOCATOR_H

#include <Arduino.h>

struct Point3D {
  double x, y, z;
  
  Point3D(double _x = 0, double _y = 0, double _z = 0) : x(_x), y(_y), z(_z) {}
  
  double distanceTo(const Point3D& other) const {
    double dx = x - other.x;
    double dy = y - other.y;
    double dz = z - other.z;
    return sqrt(dx*dx + dy*dy + dz*dz);
  }
};

struct LocationResult {
  bool success;
  Point3D position;
  double error;
  int iterations;
};

class TDOALocator {
public:
  TDOALocator(double sound_speed = 343.0);
  
  void setMicPositions(double x1, double y1, double z1,
                       double x2, double y2, double z2,
                       double x3, double y3, double z3,
                       double x4, double y4, double z4);
  
  // 定位
  LocationResult locate(int64_t t1_us, int64_t t2_us, int64_t t3_us, int64_t t4_us);
  
  void setSoundSpeed(double speed) { _sound_speed = speed; }
  void setMaxIterations(int max_iter) { _max_iterations = max_iter; }
  void setConvergenceThreshold(double threshold) { _convergence_threshold = threshold; }
  void setInitialOffset(double offset) { _initial_offset = offset; }
  void setForcePositiveZ(bool force) { _force_positive_z = force; }

private:
  double _sound_speed;
  Point3D _mics[4];
  int _max_iterations;
  double _convergence_threshold;
  double _initial_offset;
  bool _force_positive_z;
  
  Point3D getMicCenter() const;
  LocationResult solve(double d21, double d31, double d41);
  LocationResult solveNewtonRaphson(double d21, double d31, double d41, Point3D init);
};

#endif