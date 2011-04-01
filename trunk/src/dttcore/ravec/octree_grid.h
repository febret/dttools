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

// The OctreeGrid implements an octree-based 3D map.
// Note that the OctNodes use the templatized type T to store child node values
// AND also use type T as an offset (or pointer) to child OctNodes.  Using at
// least <T = int> is highly recommended, or the maximum number of OctNodes
// will be pretty small.

#ifndef _RAVEC_OCTREE_GRID_H_
#define _RAVEC_OCTREE_GRID_H_

#include <assert.h>
#include <math.h>
#include <limits>
#include <string>
#include <vector>

#include "grid.h"

namespace ravec {

template<typename T>
class OctreeGrid : public Grid<T> {
 public:

  struct OctNode {
    // Depending on the bitmask, each octant is either the value of the octant
    // cell, or an offset (pointer) to the child OctNode.
    T octant_[8];
    // Each bit of the mask indicates whether the corresponding octant field is
    // a value (bit = 0) or a pointer to an OctNode (bit = 1).
    char bitmask_;

    OctNode(int initial_value) : bitmask_(0) {
      for (int i = 0; i < 8; ++i) octant_[i] = initial_value;
    }

    // Octant accessors
    T operator()(int i) const {
      return octant_[i];
    }

    T& operator()(int i) {
      return octant_[i];
    }

    // Check if a octant is an OctNode or just a value.
    bool IsNode(int i) const {
      return (bitmask_ & (1 << i)) != 0;
    }

    // Set a octant to be an OctNode.
    void SetNode(int i, T octant_offset) {
      bitmask_ |= (1 << i);
      octant_[i] = octant_offset;
    }

    // Set a octant to be a value.
    void SetValue(int i, T octant_value) {
      if (bitmask_ & (1 << i)) bitmask_ ^= (1 << i);
      octant_[i] = octant_value;
    }
  }; // __attribute((__packed__));

  OctreeGrid(int size_x, int size_y, int size_z, T initial_value)
    : Grid<T>(size_x, size_y, size_z, initial_value) {
    assert(size_x == size_y && size_x == size_z);
    // Create the root octnode.
    NewOctNode();
  }
  ~OctreeGrid() {};

  // Compute the OctNode child index and shift the coordinates.
  int oct_index(int dim, int *x, int *y, int *z) const {
    int i = 0;
    if (*x >= dim) {*x -= dim; i |= 1;}
    if (*y >= dim) {*y -= dim; i |= 2;}
    if (*z >= dim) {*z -= dim; i |= 4;}
    return i;
  }

  void oct_coord(int i, int dim, int *x, int *y, int *z) const {
    if (i & 1) *x += dim;
    if (i & 2) *y += dim;
    if (i & 4) *z += dim;
  }

  // Basic accessors.
  T Get(int x, int y, int z) const {
    if (x >= Grid<T>::size_x_ || y >= Grid<T>::size_y_ ||
        z >= Grid<T>::size_z_ || x < 0 || y < 0 || z < 0) {
      fprintf(stderr, "Access out of bounds (size %d %d %d, access %d %d %d)\n",
              Grid<T>::size_x_, Grid<T>::size_y_, Grid<T>::size_z_, x, y, z);
      exit(-1);
    }
    T node_index = 0;
    for (int dim = Grid<T>::size_x_ / 2; dim >= 1; dim /= 2) {
      int i = oct_index(dim, &x, &y, &z);
      if (GetOctNode(node_index).IsNode(i)) {
        node_index = GetOctNode(node_index)(i);
      } else {
        T val = GetOctNode(node_index)(i);
		return val;
      }
    }
    return Grid<T>::initial_value_;
  }

  T Set(int x, int y, int z, T value) {
    if (x >= Grid<T>::size_x_ || y >= Grid<T>::size_y_ ||
        z >= Grid<T>::size_z_ || x < 0 || y < 0 || z < 0) {
      fprintf(stderr, "Access out of bounds\n");
      exit(-1);
    }
    T node_index = 0;
    int i;
    for (int dim = Grid<T>::size_x_ / 2; dim >= 2; dim /= 2) {
      i = oct_index(dim, &x, &y, &z);
      if (GetOctNode(node_index).IsNode(i)) {
        node_index = GetOctNode(node_index)(i);
      } else {
        // Create a new child, update the parent, and tail-recurse.
        T child_index = (T)NewOctNode();
        GetOctNode(node_index).SetNode(i, child_index);
        node_index = child_index;
      }
    }
    i = oct_index(1, &x, &y, &z);
    GetOctNode(node_index).SetValue(i, value);
    return GetOctNode(node_index)(i);
  }

  // Draws a line from start (x,y,z) to end (x,y,z), setting all the cells on
  // the line to value.
  // Returns the length of the line.
  double SetLine(int start_x, int start_y, int start_z,
                 int end_x, int end_y, int end_z, T value) {
    return B3dCore<&OctreeGrid<T>::b3d_set, true>(start_x, start_y, start_z,
                                                  end_x, end_y, end_z, value);
  }

  // Draws a line from start (x,y,z) to end (x,y,z), adding value to each cell.
  // The resulting cell value should be clipped to T_MAX and T_MIN.
  // Returns the length of the line.
  double AddLine(int start_x, int start_y, int start_z,
                 int end_x, int end_y, int end_z, T value) {
    return B3dCore<&OctreeGrid<T>::b3d_add, true>(start_x, start_y, start_z,
                                                  end_x, end_y, end_z, value);
  }

  // Cast a line from start (x,y,z) to end (x,y,z), stopping if the value of a
  // cell ever exceeds value.
  // Returns the distance from the start to the threshold-exceeding cell.  If
  // the first cell exceeds the threshold, returns 0, if the line reaches the
  // end of the map, returns -1.
  double CastLine(int start_x, int start_y, int start_z,
                  int end_x, int end_y, int end_z, T value) {
    return B3dCore<&OctreeGrid<T>::b3d_cast, false>(start_x, start_y, start_z,
                                                    end_x, end_y, end_z, value);
  }

  int Size() {
    return data_.size() * sizeof(data_[0]);
  }

  // Note:  Save and Load ignore endian, etc., issues.
  void Save(const std::string &filename) {
    FILE *fp;
    fp = fopen(filename.c_str(), "wb");
    assert(fp);
    size_t items_written;
    int size = Grid<T>::size_x_;
    items_written = fwrite(&size, sizeof(int), 1, fp);
    items_written = fwrite(&size, sizeof(int), 1, fp);
    items_written = fwrite(&size, sizeof(int), 1, fp);
    fwrite(&(data_[0]), sizeof(data_[0]), data_.size(), fp);
    fclose(fp);
  }

  void Load(const std::string &filename) {
    data_.clear();
    FILE *fp;
    fp = fopen(filename.c_str(), "rb");
    assert(fp);
    size_t items_read;
    int size;
    items_read = fread(&size, sizeof(int), 1, fp);
    items_read = fread(&size, sizeof(int), 1, fp);
    items_read = fread(&size, sizeof(int), 1, fp);
    Grid<T>::size_x_ = size;
    Grid<T>::size_y_ = size;
    Grid<T>::size_z_ = size;
    while (!feof(fp)) {
      OctNode node(0);
      items_read = fread(&node, sizeof(OctNode), 1, fp);
      if (items_read == 1) AddOctNode(node);
    }
    fclose(fp);
  }

  // TODO compact
  // TODO convert to uniform
  // TODO copy
  // TODO merge
  // TODO entropy

 protected:
  // Octree storage and accessors.
  std::vector<OctNode> data_;

  // Get an OctNode from its index.
  OctNode GetOctNode(T i) const { return data_[i]; }
  OctNode& GetOctNode(T i) { return data_[i]; }

  size_t NewOctNode() {
    OctNode node(Grid<T>::initial_value_);
    data_.push_back(node);
    return data_.size() - 1;
  }

  size_t AddOctNode(const OctNode &node) {
    data_.push_back(node);
    return data_.size() - 1;
  }

  // Bresenham 3D core functions plug into B3dCore (as template arguments).
  // Return false to break the main B3D loop.
  bool b3d_set(T node_index, int octant, T value) {
    GetOctNode(node_index).SetValue(octant, value);
    return true;
  }

  bool b3d_add(T node_index, int octant, T value) {
    T v = GetOctNode(node_index)(octant);
    const T v1 = v + value;
    // Deal with over/underflow.
    if (value < 0 && v1 > v) {
      v = std::numeric_limits<T>::max();
    } else if (value > 0 && v1 < v) {
      v = std::numeric_limits<T>::min();
    } else {
      v = v1;
    }
    GetOctNode(node_index).SetValue(octant, v);
    return true;
  }

  bool b3d_cast(T node_index, int octant, T value) {
    if (GetOctNode(node_index)(octant) > value) {
      return false;
    }
    return true;
  }

  template<bool (OctreeGrid<T>::*F)(T node_index, int octant, T value),
           bool create_on_demand>
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

    T parents[32];                                                    
    int octants[32];
    int level = 0;
    T node_index = 0;
    parents[0] = node_index;

    // Three cases, depending on the line slope:
    int *a, *b, *c;
    int D;
    int Da2, Db2, Dc2;
    int a_inc, b_inc, c_inc;
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
    int dim = Grid<T>::size_x_ >> 1;
    int err_b = Db2 - D;
    int err_c = Dc2 - D;
    bool out_of_bounds = false;
    // The main B3D loop.
    int distI = 0;
    while (distI < D) {
      // Descend the octree as necessary
      while (x >= dim << 1 || y >= dim << 1 || z >= dim << 1 ||
             x < 0 || y < 0 || z < 0) {
        dim = dim << 1;
        if (--level < 0) {
          out_of_bounds = true;
          break;
        }
        node_index = parents[level];
        int i = octants[level];
        oct_coord(i, dim, &x, &y, &z);
      }
      if (out_of_bounds) break;
      // Ascend the octree as far as possible
      int tx = x;
      int ty = y;
      int tz = z;
      int i = oct_index(dim, &tx, &ty, &tz);
      while (GetOctNode(node_index).IsNode(i)) {
        octants[level++] = i;
        dim = dim >> 1;
        node_index = GetOctNode(node_index)(i);
        parents[level] = node_index;
        x = tx;
        y = ty;
        z = tz;
        i = oct_index(dim, &tx, &ty, &tz);
      }
      if (create_on_demand) {
        // Generate octants as needed
        while (dim > 1) {
          T child_index = NewOctNode();
          GetOctNode(node_index).SetNode(i, child_index);
          octants[level++] = i;
          dim = dim >> 1;
          node_index = child_index;
          parents[level] = node_index;
          x = tx;
          y = ty;
          z = tz;
          i = oct_index(dim, &tx, &ty, &tz);
        }
      }
      // Call the core function.
      if (!(this->*F)(node_index, i, value)) break;
      // Perform a Bresenham step to update the coordinates.
      while (err_b > 0) {
        *b += b_inc;
        err_b -= Da2;
      }
      while (err_c > 0) {
        *c += c_inc;
        err_c -= Da2;
      }
      // TODO: dynamic step size
      err_b += Db2;
      err_c += Dc2;
      *a += a_inc;
      distI++;
    }

    // Pop back up the octree, and reconstruct the coordinates.
    while (level-- >= 1) {
      dim = dim << 1;
      int i = octants[level];
      oct_coord(i, dim, &x, &y, &z);
    }
    double distance = sqrt((double)(x - start_x) * (x - start_x) +
                           (y - start_y) * (y - start_y) +
                           (z - start_z) * (z - start_z));
    // Return -distance if the bounds were exceeded.
    if (out_of_bounds) return -distance;
    return distance;
  }
};

}  // namespace ravec

#endif  // _RAVEC_OCTREE_GRID_H_
