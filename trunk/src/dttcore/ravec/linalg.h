/*
    Copyright 2010 Nathaniel Fairfield <than@timbrel.org>

    This file is part of Ravec.

    Ravec is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Ravec is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Ravec.  If not, see <http://www.gnu.org/licenses/>.
*/

// Basic linear algebra for transformations, including 4x4 matrices, 4 vectors,
// and 6 degree-of-freedom poses.

#ifndef _RAVEC_LINALG_H_
#define _RAVEC_LINALG_H_

#include <math.h>
#include <string.h>
#include <iostream>

namespace ravec {

class Vector;
class Transform;
class Pose;

// 4 vector class
class Vector {
 public:
  Vector(double val = 0) { for (int i = 0; i < 4; ++i) V_[i] = val; }

  Vector(double x, double y, double z, double w) {
    V_[0] = x;
    V_[1] = y;
    V_[2] = z;
    V_[3] = w;
  }

  Vector(const Vector& v) { memcpy(V_, v.V_, 4 * sizeof(double)); }

  Vector(const double* d) { memcpy(V_, d, 4 * sizeof(double)); }

  double operator()(unsigned r) const { return V_[r]; }
  double& operator()(unsigned r) { return V_[r]; }

  // In-place dehomogenization.
  void Dehomogenize() {
    V_[0] /= V_[3];
    V_[1] /= V_[3];
    V_[2] /= V_[3];
    V_[3] = 1.0;
  }

  friend std::ostream& operator<<(std::ostream& output, const Vector& V) {
    for (int i = 0; i < 4; ++i) {
      output << V.V_[i] << " ";
    }
    return output;
  }

  double V_[4];
};

// Row-major 4x4 transformation matrix.
class Transform {
 public:
  Transform() { 
    memset(T_, 0, 16 * sizeof(double)); 
  }

  Transform(double diag) { 
    memset(T_, 0, 16 * sizeof(double)); 
    for (int i = 0; i < 4; ++i) T_[i * 4 + i] = diag;
  }

  Transform(const Transform& T) { 
    memcpy(T_, T.T_, 16 * sizeof(double)); 
  }

  // d must be an array of length 16.
  Transform(const double* d) { 
    memcpy(T_, d, 16 * sizeof(double)); 
  }

  double& operator()(unsigned r, unsigned c) { return T_[r * 4 + c]; }
  double operator()(unsigned r, unsigned c) const { return T_[r * 4 + c]; }

  // Matrix-matrix multiplication
  void Multiply(const Transform& B, Transform* C) const {
    for (int i = 0;  i < 16; i += 4) {
      for (int j = 0; j < 4; ++j) {
        C->T_[i + j] = B.T_[i + 0] * T_[j + 0] +
                       B.T_[i + 1] * T_[j + 4] +
                       B.T_[i + 2] * T_[j + 8] +
                       B.T_[i + 3] * T_[j + 12];
      } 
    }
  }

  // Matrix-vector multiplication
  void Multiply(const Vector& B, Vector* C) const {
    C->V_[0] = T_[0] * B(0) + T_[1] * B(1) + T_[2] * B(2) + T_[3] * B(3);
    C->V_[1] = T_[4] * B(0) + T_[5] * B(1) + T_[6] * B(2) + T_[7] * B(3);
    C->V_[2] = T_[8] * B(0) + T_[9] * B(1) + T_[10] * B(2) + T_[11] * B(3);
    C->V_[3] = T_[12] * B(0) + T_[13] * B(1) + T_[14] * B(2) + T_[15] * B(3);
  }

  Transform operator* (const Transform& B) const {
    Transform C;
    this->Multiply(B, &C);
    return C;
  }

  Vector operator* (const Vector& B) const {
    Vector C;
    this->Multiply(B, &C);
    return C;
  }

  // Fast in-place inverse for transformation matrices.
  void Invert() { 
    double tmp;
    tmp = T_[1];
    T_[1] = T_[4];
    T_[4] = tmp;
    tmp = T_[2];
    T_[2] = T_[8];
    T_[8] = tmp;
    tmp = T_[6];
    T_[6] = T_[9];
    T_[9] = tmp;
    double x = -T_[0] * T_[3] - T_[1] * T_[7] - T_[2] * T_[11];
    double y = -T_[4] * T_[3] - T_[5] * T_[7] - T_[6] * T_[11];
    double z = -T_[8] * T_[3] - T_[9] * T_[7] - T_[10] * T_[11];
    T_[3] = x;
    T_[7] = y;
    T_[11] = z;
  }

  Transform Inverse() const {
    Transform A(*this);
    A.Invert();
    return A;
  }

  Pose ToPose() const;

  // In-place transpose.
  void Transpose() {
    double tmp;
    for (int i = 0; i < 4; ++i) {
      for (int j = i + 1; j < 4; ++j) {
        tmp = (*this)(i, j);
        (*this)(i, j) = (*this)(j, i);
        (*this)(j, i) = tmp;
      }
    }
  }

  friend std::ostream& operator<<(std::ostream& output, const Transform& T) {
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        output << T.T_[i * 4 + j] << " ";
      }
      output << std::endl;
    }
    return output;
  }

  double T_[16];  
};


// Pose class wraps 6 DOF poses with 4x4 transformation matrix operations
class Pose {
 public: 
  Pose() { memset(P_, 0, 6 * sizeof(double)); }

  Pose(double c[6]) { memcpy(P_, c, 6 * sizeof(double)); }

  Pose(const Pose& P) { memcpy(P_, P.P_, 6 * sizeof(double)); }

  Pose(double roll, double pitch, double yaw, double x, double y, double z) {
    P_[0] = roll;
    P_[1] = pitch;
    P_[2] = yaw;
    P_[3] = x;
    P_[4] = y;
    P_[5] = z;
  }

  double& operator()(unsigned i) { return P_[i]; }
  double operator()(unsigned i) const { return P_[i]; }

  // Named convenience accessors.
  double roll() const { return P_[0]; }
  double pitch() const { return P_[1]; }
  double yaw() const { return P_[2]; }
  double x() const { return P_[3]; }
  double y() const { return P_[4]; }
  double z() const { return P_[5]; }

  Transform ToTransform() const;

  friend std::ostream& operator<<(std::ostream& output, const Pose& P) {
    for (int i = 0; i < 6; ++i) {
      output << P.P_[i] << " ";
    }
    return output;
  };

  double P_[6];
};

inline Pose Transform::ToPose() const {
  Pose P;
  // Order is XYZ.T
  P.P_[0] = atan2(T_[9], T_[10]);
  P.P_[1] = -asin(T_[8]);
  P.P_[2] = atan2(T_[4], T_[0]);
  P.P_[3] = T_[3]/T_[15];
  P.P_[4] = T_[7]/T_[15];
  P.P_[5] = T_[11]/T_[15];
  return P;
}

inline Transform Pose::ToTransform() const {
  Transform T;
  double cr, sr, cp, sp, cy, sy;
  cr = cos(P_[0]); sr = sin(P_[0]);
  cp = cos(P_[1]); sp = sin(P_[1]);
  cy = cos(P_[2]); sy = sin(P_[2]);
  // Order is XYZ.T
  T(0, 0) = cy*cp; T(0, 1) = -sy*cr+cy*sp*sr; T(0, 2) =  sy*sr+cy*sp*cr;
  T(1, 0) = sy*cp; T(1, 1) =  cy*cr+sy*sp*sr; T(1, 2) = -cy*sr+sy*sp*cr;
  T(2, 0) =   -sp; T(2, 1) =           cp*sr; T(2, 2) =           cp*cr;
  T(0, 3) = P_[3];
  T(1, 3) = P_[4];
  T(2, 3) = P_[5];
  T(3, 0) = 0;
  T(3, 1) = 0;
  T(3, 2) = 0;
  T(3, 3) = 1;
  return T;
};

}  // namespace ravec

#endif  // _RAVEC_LINALG_H_
