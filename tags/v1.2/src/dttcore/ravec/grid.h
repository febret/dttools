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

// Base class for a 3D grid-based map.
//
// Example usage:
//   unsigned char initial_value = 0;
//   MyGrid G(256, 256, 256, initial_value);
//   unsigned char my_value = G(0, 0, 0);
//   // my_value should be initial_value
//   unsigned char new_value = 5;
//   G.Set(0, 0, 0, new_value);
//   // G(0, 0, 0) should be new_value
//   G.SetLine(0, 0, 0, 10, 10, 10, new_value);
//   // G(1, 1, 1), etc., should be new_value
//   G.AddLine(0, 0, 0, 10, 0, 0, new_value);
//   // G(0, 0, 0) should be 2*new_value
//   // G(1, 0, 0), etc., should be new_value
//   G.Set(10, 0, 0, 100);
//   double distance = G.CastLine(0, 0, 0, 100, 0, 0, 50);
//   // distance should be 10

#ifndef _RAVEC_GRID_H_
#define _RAVEC_GRID_H_

// The Grid base class defines the common elements of a 3D grid-based map.

namespace ravec {

template<typename T>
class Grid {
 public:
  Grid(int size_x, int size_y, int size_z, T initial_value)
    : size_x_(size_x),
      size_y_(size_y),
      size_z_(size_z),
      initial_value_(initial_value) {}
  virtual ~Grid() {}

  // Basic accessors.
  virtual T Get(int x, int y, int z) const = 0;
  virtual T Set(int x, int y, int z, T value) = 0;
  virtual T Add(int x, int y, int z, T value) {
    return Set(x, y, z, value + Get(x, y, z));
  }
  // Convenience Get operator.
  virtual T operator()(int x, int y, int z) const { return Get(x, y, z); }
  // Special 4 argument Get ignores the last argument, but is useful for
  // templatized functions (over Get/Set/Add).
  // NOTE: not const (though the underlying Get is)
  virtual T Get(int x, int y, int z, T value) { return Get(x, y, z); }

  // Draws a line from start (x,y,z) to end (x,y,z), setting all the cells on
  // the line to value.
  // Returns the length of the line.
  virtual double SetLine(int start_x, int start_y, int start_z,
                         int end_x, int end_y, int end_z, T value) = 0;

  // Draws a line from start (x,y,z) to end (x,y,z), adding value to each cell.
  // The resulting cell value should be clipped to T_MAX, and T_MIN.
  // Returns the length of the line.
  virtual double AddLine(int start_x, int start_y, int start_z,
                         int end_x, int end_y, int end_z, T value) = 0;

  // Cast a line from start (x,y,z) to end (x,y,z), stopping if the value of a
  // cell ever exceeds value.
  // Returns the distance from the start to the threshold-exceeding cell.  If
  // the first cell exceeds the threshold, returns 0, if the line reaches the
  // end of the map, returns -1.
  virtual double CastLine(int start_x, int start_y, int start_z,
                          int end_x, int end_y, int end_z, T value) = 0;

  int size_x() const { return size_x_; }
  int size_y() const { return size_y_; }
  int size_z() const { return size_z_; }

 protected:
  int size_x_;
  int size_y_;
  int size_z_;
  T initial_value_;
};

}  // namespace ravec

#endif  // _RAVEC_GRID_H_
