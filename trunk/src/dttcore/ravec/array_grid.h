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

// The ArrayGrid implements a uniform array-based 3D map.

#ifndef _RAVEC_ARRAY_GRID_H_
#define _RAVEC_ARRAY_GRID_H_

#include "grid.h"
#include <limits>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

namespace ravec {

template<typename T>
class ArrayGrid : public Grid<T> {
 public:
  ArrayGrid(int size_x, int size_y, int size_z, T initial_value)
    : Grid<T>(size_x, size_y, size_z, initial_value) {
    data_ = new T[size_x * size_y * size_z];
    // Set initial value.
    for (int i = 0; i < size_x * size_y * size_z; ++i) {
      data_[i] = initial_value;
    }
  }

  ~ArrayGrid() {
    delete[] data_;
  }

  // Basic accessors.
  T Get(int x, int y, int z) const {
    return data_[((x * Grid<T>::size_y_) + y) * Grid<T>::size_z_ + z];
  }
  T Set(int x, int y, int z, T value) {
    return data_[((x * Grid<T>::size_y_) + y) * Grid<T>::size_z_ + z] = value;
  }

  // The following methods use a 3D version of Bresenham's line drawing
  // algorithm.

  // Draw a line from start (x,y,z) to end (x,y,z), setting all the cells on
  // the line to value.
  // Returns the length of the line.
  double SetLine(int start_x, int start_y, int start_z,
                 int end_x, int end_y, int end_z, T value) {
    return B3dCore<&ArrayGrid<T>::b3d_set>(start_x, start_y, start_z,
                                           end_x, end_y, end_z, value);
  }

  // Draw a line from start (x,y,z) to end (x,y,z), adding value to each cell.
  // The resulting cell value should be clipped to T_MAX, and T_MIN.
  // Returns the length of the line.
  double AddLine(int start_x, int start_y, int start_z,
                 int end_x, int end_y, int end_z, T value) {
    return B3dCore<&ArrayGrid<T>::b3d_add>(start_x, start_y, start_z,
                                           end_x, end_y, end_z, value);
  }

  // Cast a line from start (x,y,z) to end (x,y,z), stopping if the value of a
  // cell ever exceeds value.
  // Returns the distance from the start to the threshold-exceeding cell.  If
  // the first cell exceeds the threshold, returns 0, if the line reaches the
  // end of the map, returns -1.
  double CastLine(int start_x, int start_y, int start_z,
                  int end_x, int end_y, int end_z, T value) {
    return B3dCore<&ArrayGrid<T>::b3d_cast>(start_x, start_y, start_z,
                                            end_x, end_y, end_z, value);
  }

 protected:
  // Array that stores the grid cell values.
  T *data_;

  // Bresenham 3D core functions plug into B3dCore (as template arguments).
  // Return false to break the main B3D loop.
  bool b3d_set(int x, int y, int z, T value) {
    this->Set(x, y, z, value);
    return true;
  }

  bool b3d_add(int x, int y, int z, T value) {
      T v = this->Get(x, y, z);
    // Deal with over/underflow.
    if (value < 0 && (v + value) > v) {
      v = std::numeric_limits<T>::max();
    } else if (value > 0 && (v + value) < v) {
      v = std::numeric_limits<T>::min();
    } else {
      v += value;
    }
    this->Set(x, y, z, v);
    return true;
  }

  bool b3d_cast(int x, int y, int z, T value) {
    if (this->Get(x, y, z) > value) {
      return false;
    }
    return true;
  }

  template <bool (ArrayGrid<T>::*F)(int x, int y, int z, T value)>
  double B3dCore(int start_x, int start_y, int start_z,
      int end_x, int end_y, int end_z, T value) {
    int x = start_x;
    int y = start_y;
    int z = start_z;
    const int dx = end_x - start_x;
    const int dy = end_y - start_y;
    const int dz = end_z - start_z;
    const int x_inc = (dx < 0) ? -1 : 1;
    const int y_inc = (dy < 0) ? -1 : 1;
    const int z_inc = (dz < 0) ? -1 : 1;
    const int Dx = abs(dx);
    const int Dy = abs(dy);
    const int Dz = abs(dz);
    const int Dx2 = Dx << 1;
    const int Dy2 = Dy << 1;
    const int Dz2 = Dz << 1;
    // a, b, c are pointers to x, y, z, with the order depending on the case.
    int *a, *b, *c;
    int D;
    int Da2, Db2, Dc2;
    int a_inc, b_inc, c_inc;
    // Three cases, depending on the line slope:
    if ((Dx >= Dy) && (Dx >= Dz)) {
      a = &x; b = &y; c = &z;
      D = Dx;
      Da2 = Dx2; Db2 = Dy2; Dc2 = Dz2;
      a_inc = x_inc; b_inc = y_inc; c_inc = z_inc;
    } else if ((Dy >= Dx) && (Dy >= Dz)) {
      a = &y; b = &x; c = &z;
      D = Dy;
      Da2 = Dy2; Db2 = Dx2; Dc2 = Dz2;
      a_inc = y_inc; b_inc = x_inc; c_inc = z_inc;
    } else {
      a = &z; b = &y; c = &x;
      D = Dz;
      Da2 = Dz2; Db2 = Dy2; Dc2 = Dx2;
      a_inc = z_inc; b_inc = y_inc; c_inc = x_inc;
    }
    int err_b = Db2 - D;
    int err_c = Dc2 - D;
    bool out_of_bounds = false;
    // The main B3D loop.
    for (int i = 0; i < D; ++i) {
      // Check bounds.
      if (x >= Grid<T>::size_x_ || y >= Grid<T>::size_y_ ||
          z >= Grid<T>::size_z_ || x < 0 || y < 0 || z < 0) {
        out_of_bounds = true;
        break;
      }
      // Call the core function.
      if (!(this->*F)(x, y, z, value)) break;
      if (err_b > 0) {
        *b += b_inc;
        err_b -= Da2;
      }
      if (err_c > 0) {
        *c += c_inc;
        err_c -= Da2;
      }
      err_b += Db2;
      err_c += Dc2;
      *a += a_inc;
    }
    double distance = sqrt((x - start_x)*(x - start_x) +
                           (y - start_y)*(y - start_y) +
                           (z - start_z)*(z - start_z));
    // Return -distance if the bounds were exceeded.
    if (out_of_bounds) return -distance;
    return distance;
  }
};

}  // namespace ravec

#endif  // _RAVEC_ARRAY_GRID_H_
