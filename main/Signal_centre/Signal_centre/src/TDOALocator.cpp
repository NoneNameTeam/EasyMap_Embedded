#include "TDOALocator.h"

TDOALocator::TDOALocator(double sound_speed) {
  _sound_speed = sound_speed;
  _max_iterations = 200;
  _convergence_threshold = 1e-7;
  _initial_offset = 0.1;
  _force_positive_z = true;
  
  _mics[0] = Point3D(0, 0, 0);
  _mics[1] = Point3D(1, 0, 0);
  _mics[2] = Point3D(1, 1, 0);
  _mics[3] = Point3D(0, 1, 0);
}

void TDOALocator::setMicPositions(double x1, double y1, double z1,
                                   double x2, double y2, double z2,
                                   double x3, double y3, double z3,
                                   double x4, double y4, double z4) {
  _mics[0] = Point3D(x1, y1, z1);
  _mics[1] = Point3D(x2, y2, z2);
  _mics[2] = Point3D(x3, y3, z3);
  _mics[3] = Point3D(x4, y4, z4);
}

Point3D TDOALocator::getMicCenter() const {
  return Point3D(
    (_mics[0].x + _mics[1].x + _mics[2].x + _mics[3].x) * 0.25,
    (_mics[0].y + _mics[1].y + _mics[2].y + _mics[3].y) * 0.25,
    (_mics[0].z + _mics[1].z + _mics[2].z + _mics[3].z) * 0.25
  );
}

LocationResult TDOALocator::locate(int64_t t1_us, int64_t t2_us, int64_t t3_us, int64_t t4_us) {
  double d21 = _sound_speed * (double)(t2_us - t1_us) * 1e-6;
  double d31 = _sound_speed * (double)(t3_us - t1_us) * 1e-6;
  double d41 = _sound_speed * (double)(t4_us - t1_us) * 1e-6;
  
  return solve(d21, d31, d41);
}

LocationResult TDOALocator::solve(double d21, double d31, double d41) {
  Point3D center = getMicCenter();
  
  Point3D inits[12] = {
    Point3D(center.x + _initial_offset, center.y, 0.2),
    Point3D(center.x - _initial_offset, center.y, 0.2),
    Point3D(center.x, center.y + _initial_offset, 0.2),
    Point3D(center.x, center.y - _initial_offset, 0.2),
    Point3D(center.x + _initial_offset, center.y, 0.6),
    Point3D(center.x - _initial_offset, center.y, 0.6),
    Point3D(center.x, center.y + _initial_offset, 0.6),
    Point3D(center.x, center.y - _initial_offset, 0.6),
    Point3D(center.x + _initial_offset, center.y, 1.2),
    Point3D(center.x - _initial_offset, center.y, 1.2),
    Point3D(center.x, center.y + _initial_offset, 1.2),
    Point3D(center.x, center.y - _initial_offset, 1.2)
  };
  
  LocationResult best;
  best.success = false;
  best.error = 1e10;
  best.iterations = 0;
  
  for (int i = 0; i < 12; i++) {
    LocationResult result = solveNewtonRaphson(d21, d31, d41, inits[i]);
    
    if (result.error < best.error) {
      if (_force_positive_z && result.position.z < 0) continue;
      best = result;
    }
  }
  
  best.success = (best.error < 0.001);
  return best;
}

LocationResult TDOALocator::solveNewtonRaphson(double d21, double d31, double d41, Point3D init) {
  LocationResult result;
  result.success = false;
  result.iterations = 0;
  
  double x = init.x;
  double y = init.y;
  double z = init.z;
  
  double best_err = 1e10;
  double best_x = x, best_y = y, best_z = z;
  
  for (int iter = 0; iter < _max_iterations; iter++) {
    result.iterations = iter + 1;
    
    double dx1 = x - _mics[0].x, dy1 = y - _mics[0].y, dz1 = z - _mics[0].z;
    double dx2 = x - _mics[1].x, dy2 = y - _mics[1].y, dz2 = z - _mics[1].z;
    double dx3 = x - _mics[2].x, dy3 = y - _mics[2].y, dz3 = z - _mics[2].z;
    double dx4 = x - _mics[3].x, dy4 = y - _mics[3].y, dz4 = z - _mics[3].z;
    
    double r1 = sqrt(dx1*dx1 + dy1*dy1 + dz1*dz1);
    double r2 = sqrt(dx2*dx2 + dy2*dy2 + dz2*dz2);
    double r3 = sqrt(dx3*dx3 + dy3*dy3 + dz3*dz3);
    double r4 = sqrt(dx4*dx4 + dy4*dy4 + dz4*dz4);
    
    if (r1 < 1e-10) r1 = 1e-10;
    if (r2 < 1e-10) r2 = 1e-10;
    if (r3 < 1e-10) r3 = 1e-10;
    if (r4 < 1e-10) r4 = 1e-10;
    
    double f1 = (r2 - r1) - d21;
    double f2 = (r3 - r1) - d31;
    double f3 = (r4 - r1) - d41;
    
    double err = sqrt(f1*f1 + f2*f2 + f3*f3);
    
    if (err < best_err) {
      best_err = err;
      best_x = x;
      best_y = y;
      best_z = z;
    }
    
    if (err < _convergence_threshold) {
      result.position = Point3D(x, y, z);
      result.error = err;
      result.success = true;
      return result;
    }
    
    double J[3][3];
    J[0][0] = dx2/r2 - dx1/r1;  J[0][1] = dy2/r2 - dy1/r1;  J[0][2] = dz2/r2 - dz1/r1;
    J[1][0] = dx3/r3 - dx1/r1;  J[1][1] = dy3/r3 - dy1/r1;  J[1][2] = dz3/r3 - dz1/r1;
    J[2][0] = dx4/r4 - dx1/r1;  J[2][1] = dy4/r4 - dy1/r1;  J[2][2] = dz4/r4 - dz1/r1;
    
    double det = J[0][0]*(J[1][1]*J[2][2] - J[1][2]*J[2][1])
               - J[0][1]*(J[1][0]*J[2][2] - J[1][2]*J[2][0])
               + J[0][2]*(J[1][0]*J[2][1] - J[1][1]*J[2][0]);
    
    if (fabs(det) < 1e-15) {
      double lr = 0.01;
      double gx = 2*(f1*(dx2/r2-dx1/r1) + f2*(dx3/r3-dx1/r1) + f3*(dx4/r4-dx1/r1));
      double gy = 2*(f1*(dy2/r2-dy1/r1) + f2*(dy3/r3-dy1/r1) + f3*(dy4/r4-dy1/r1));
      double gz = 2*(f1*(dz2/r2-dz1/r1) + f2*(dz3/r3-dz1/r1) + f3*(dz4/r4-dz1/r1));
      x -= lr * gx;
      y -= lr * gy;
      z -= lr * gz;
    } else {
      double inv[3][3];
      inv[0][0] = (J[1][1]*J[2][2] - J[1][2]*J[2][1]) / det;
      inv[0][1] = (J[0][2]*J[2][1] - J[0][1]*J[2][2]) / det;
      inv[0][2] = (J[0][1]*J[1][2] - J[0][2]*J[1][1]) / det;
      inv[1][0] = (J[1][2]*J[2][0] - J[1][0]*J[2][2]) / det;
      inv[1][1] = (J[0][0]*J[2][2] - J[0][2]*J[2][0]) / det;
      inv[1][2] = (J[0][2]*J[1][0] - J[0][0]*J[1][2]) / det;
      inv[2][0] = (J[1][0]*J[2][1] - J[1][1]*J[2][0]) / det;
      inv[2][1] = (J[0][1]*J[2][0] - J[0][0]*J[2][1]) / det;
      inv[2][2] = (J[0][0]*J[1][1] - J[0][1]*J[1][0]) / det;
      
      double delta_x = -(inv[0][0]*f1 + inv[0][1]*f2 + inv[0][2]*f3);
      double delta_y = -(inv[1][0]*f1 + inv[1][1]*f2 + inv[1][2]*f3);
      double delta_z = -(inv[2][0]*f1 + inv[2][1]*f2 + inv[2][2]*f3);
      
      double step = sqrt(delta_x*delta_x + delta_y*delta_y + delta_z*delta_z);
      if (step > 0.5) {
        double scale = 0.5 / step;
        delta_x *= scale;
        delta_y *= scale;
        delta_z *= scale;
      }
      
      x += delta_x;
      y += delta_y;
      z += delta_z;
    }
    
    if (_force_positive_z && z < 0.001) z = 0.001;
    
    x = fmax(-20.0, fmin(20.0, x));
    y = fmax(-20.0, fmin(20.0, y));
    z = fmin(20.0, z);
  }
  
  result.position = Point3D(best_x, best_y, best_z);
  result.error = best_err;
  result.success = (best_err < 0.001);
  
  return result;
}