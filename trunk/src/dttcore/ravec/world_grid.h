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

#ifndef _RAVEC_WORLDGRID_H_
#define _RAVEC_WORLDGRID_H_

// The WorldGrid class wraps together a Grid object and a world frame -> map
// frame transform so that the user only needs to deal with world frame or body
// frame coordinates.
#include "grid.h"
#include "linalg.h"

#define round(x) ((x)>=0?(int)((x)+0.5):(int)((x)-0.5))

namespace ravec {

template<typename T>
class WorldGrid {
 public:
  // The caller maintains ownership of grid.
  WorldGrid(const Transform &world_transform, Grid<T> *grid)
    : world_transform_(world_transform),
      grid_(grid) {};
  ~WorldGrid() {};

  // World-frame operations transform the coordinates from world frame to map
  // frame.

  // Basic world-frame operations
  template<T (Grid<T>::*F)(int, int, int, T)>
  T BasicWorld(double start_x, double start_y, double start_z, T value) {
    int sx, sy, sz;
    WorldToMap(start_x, start_y, start_z, &sx, &sy, &sz);
    return (grid_->*F)(sx, sy, sz, value);
  }

  T GetWorld(double x, double y, double z) { // const
    return BasicWorld<&Grid<T>::Get>(x, y, z, 0);
  }
  T SetWorld(double x, double y, double z, T value) { 
    return BasicWorld<&Grid<T>::Set>(x, y, z, value);
  }
  T AddWorld(double x, double y, double z, T value) {
    return BasicWorld<&Grid<T>::Add>(x, y, z, value);
  }

  // Line world-frame operations
  template<double (Grid<T>::*F)(int, int, int, int, int, int, T)>
  double LineWorld(double start_x, double start_y, double start_z,
                   double end_x, double end_y, double end_z, T value) {
    int sx, sy, sz, ex, ey, ez;
    WorldToMap(start_x, start_y, start_z, &sx, &sy, &sz);
    WorldToMap(end_x, end_y, end_z, &ex, &ey, &ez);
    return (grid_->*F)(sx, sy, sz, ex, ey, ez, value) * world_transform_(3, 3);
  }
  template<double (Grid<T>::*F)(int, int, int, int, int, int, T)>
  double LineWorldDirection(double start_x, double start_y, double start_z,
                            double dx, double dy, double dz,
                            double distance, T value) {
    int sx, sy, sz, ex, ey, ez;
    double end_x, end_y, end_z;
    WorldToMap(start_x, start_y, start_z, &sx, &sy, &sz);
    StartDirectionDistanceToEnd(start_x, start_y, start_z, dx, dy, dz,
                                distance, &end_x, &end_y, &end_z);
    WorldToMap(end_x, end_y, end_z, &ex, &ey, &ez);
    return (grid_->*F)(sx, sy, sz, ex, ey, ez, value) * world_transform_(3, 3);
  }

  double SetLineWorld(double sx, double sy, double sz,
                      double ex, double ey, double ez, T value) {
    return LineWorld<&Grid<T>::SetLine>(sx, sy, sz, ex, ey, ez, value);
  }
  double SetLineDirectionWorld(double sx, double sy, double sz,
                          double dx, double dy, double dz,
                          double distance, T value) {
    return LineWorldDirection<&Grid<T>::SetLine>(sx, sy, sz, dx, dy, dz, 
                                                 distance,  value);
  }

  double AddLineWorld(double sx, double sy, double sz,
                      double ex, double ey, double ez, T value) {
    return LineWorld<&Grid<T>::AddLine>(sx, sy, sz, ex, ey, ez, value);
  }
  double AddLineDirectionWorld(double sx, double sy, double sz,
                               double dx, double dy, double dz,
                               double distance, T value) {
    return LineWorldDirection<&Grid<T>::AddLine>(sx, sy, sz, dx, dy, dz, 
                                                 distance,  value);
  }

  double CastLineWorld(double sx, double sy, double sz,
                       double ex, double ey, double ez, T value) { // const
    return LineWorld<&Grid<T>::CastLine>(sx, sy, sz, ex, ey, ez, value);
  }
  double CastLineDirectionWorld(double sx, double sy, double sz,
                                double dx, double dy, double dz,
                                double distance, T value) { // const
    return LineWorldDirection<&Grid<T>::CastLine>(sx, sy, sz, dx, dy, dz, 
                                                  distance,  value);
  }

  // Body-frame operations use the current body pose to transform the given
  // coordinates from body frame to map frame.

  // Basic body-frame operations
  template<T (Grid<T>::*F)(int, int, int, T)>
  double BasicBody(const Transform &body_transform,
                   double start_x, double start_y, double start_z, T value) {
    int sx, sy, sz;
    BodyToMap(body_transform, start_x, start_y, start_z, &sx, &sy, &sz);
    return (grid_->*F)(sx, sy, sz, value);
  }

  T GetBody(const Transform &body_transform,
            double x, double y, double z) { // const
    return BasicBody<&Grid<T>::Get>(body_transform, x, y, z, 0);
  }
  T SetBody(const Transform &body_transform,
            double x, double y, double z, T value) { 
    return BasicBody<&Grid<T>::Set>(body_transform, x, y, z, value);
  }
  T AddBody(const Transform &body_transform,
            double x, double y, double z, T value) {
    return BasicBody<&Grid<T>::Add>(body_transform, x, y, z, value);
  }

  // Line body-frame operations
  template<double (Grid<T>::*F)(int, int, int, int, int, int, T)>
  double LineBody(const Transform &body_transform,
                  double start_x, double start_y, double start_z,
                  double end_x, double end_y, double end_z, T value) {
    int sx, sy, sz, ex, ey, ez;
    BodyToMap(body_transform, start_x, start_y, start_z, &sx, &sy, &sz);
    BodyToMap(body_transform, end_x, end_y, end_z, &ex, &ey, &ez);
    return (grid_->*F)(sx, sy, sz, ex, ey, ez, value) *
      world_transform_(3, 3) * body_transform(3, 3);
  }
  template<double (Grid<T>::*F)(int, int, int, int, int, int, T)>
  double LineBodyDirection(const Transform &body_transform,
                           double start_x, double start_y, double start_z,
                           double dx, double dy, double dz,
                           double distance, T value) {
    int sx, sy, sz, ex, ey, ez;
    double end_x, end_y, end_z;
    BodyToMap(body_transform, start_x, start_y, start_z, &sx, &sy, &sz);
    StartDirectionDistanceToEnd(start_x, start_y, start_z, dx, dy, dz,
                                distance, &end_x, &end_y, &end_z);
    WorldToMap(end_x, end_y, end_z, &ex, &ey, &ez);
    return (grid_->*F)(sx, sy, sz, ex, ey, ez, value) * world_transform_(3, 3);
      world_transform_(3, 3) * body_transform(3, 3);
  }

  double SetLineBody(const Transform &body_transform,
                     double sx, double sy, double sz,
                     double ex, double ey, double ez, T value) {
    return LineBody<&Grid<T>::SetLine>(body_transform,
                                       sx, sy, sz, ex, ey, ez, value);
  }
  double SetLineDirectionBody(const Transform &body_transform,
                              double sx, double sy, double sz,
                              double dx, double dy, double dz,
                              double distance, T value) {
    return LineBodyDirection<&Grid<T>::SetLine>(body_transform,
                                                sx, sy, sz, dx, dy, dz, 
                                                distance, value);
  }

  double AddLineBody(const Transform &body_transform,
                     double sx, double sy, double sz,
                     double ex, double ey, double ez, T value) {
    return LineBody<&Grid<T>::AddLine>(body_transform,
                                       sx, sy, sz, ex, ey, ez, value);
  }
  double AddLineDirectionBody(const Transform &body_transform,
                              double sx, double sy, double sz,
                              double dx, double dy, double dz,
                              double distance, T value) {
    return LineBodyDirection<&Grid<T>::AddLine>(body_transform,
                                                sx, sy, sz, dx, dy, dz, 
                                                distance, value);
  }

  double CastLineBody(const Transform &body_transform,
                      double sx, double sy, double sz,
                      double ex, double ey, double ez, T value) { // const
    return LineBody<&Grid<T>::CastLine>(body_transform,
                                       sx, sy, sz, ex, ey, ez, value);
  }
  double CastLineDirectionBody(const Transform &body_transform,
                               double sx, double sy, double sz,
                               double dx, double dy, double dz,
                               double distance, T value) { // const
    return LineBodyDirection<&Grid<T>::CastLine>(body_transform,
                                                 sx, sy, sz, dx, dy, dz, 
                                                 distance, value);
  }

 public:
  void WorldToMap(double xw, double yw, double zw,
                  int *xm, int *ym, int *zm) const {
    Vector v(xw, yw, zw, 1.0);
    Vector v1 = world_transform_ * v;
    v1.Dehomogenize();
    *xm = round(v1(0));
    *ym = round(v1(1));
    *zm = round(v1(2));
  }

  void BodyToMap(const Transform &body_transform,
                 double xb, double yb, double zb,
                 int *xm, int *ym, int *zm) const {
    Vector v(xb, yb, zb, 1.0);
    Vector v1 = world_transform_ * (body_transform * v);
    v1.Dehomogenize();
    *xm = round(v1(0));
    *ym = round(v1(1));
    *zm = round(v1(2));
  }

  void StartDirectionDistanceToEnd(double sx, double sy, double sz,
                                   double dx, double dy, double dz,
                                   double distance,
                                   double *ex, double *ey, double *ez) const {
    // Normalize the direction vector and project to the end
    const double norm = sqrt(dx * dx + dy * dy + dz * dz);
    *ex = round(sx + dx / norm * distance);
    *ey = round(sy + dy / norm * distance);
    *ez = round(sz + dz / norm * distance);
  }

  Grid<T> *grid_;
  Transform world_transform_;
};

}  // namespace ravec

#endif  // _RAVEC_WORLDGRID_H_
