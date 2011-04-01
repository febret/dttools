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

#ifndef _RAVEC_UTIL_H_
#define _RAVEC_UTIL_H_

#include <stdio.h>
#include <string>
#include <vector>

namespace ravec {
  // Write and read for an ND array file storage format.
  template<typename T>
  void WriteArray(const std::string &filename, const T *data,
      const std::vector<size_t> &dims) {
    FILE *fp = fopen(filename.c_str(), "wb");
    size_t items_written;
    size_t dims_size = dims.size();
    items_written = fwrite(&(dims_size), sizeof(dims_size), 1, fp);
    items_written = fwrite(&(dims[0]), sizeof(size_t), dims_size, fp);
    size_t size = 1;
    for (size_t i = 0; i < dims.size(); ++i) size *= dims[i];
    fwrite(data, sizeof(T), size, fp);
    fclose(fp);
  }

  template<typename T>
  T* ReadArray(const std::string &filename, std::vector<size_t> *dims) {
    dims->clear();
    FILE *fp = fopen(filename.c_str(), "rb");
    size_t items_read;
    size_t dims_size;
    items_read = fread(&dims_size, sizeof(size_t), 1, fp);
    dims->resize(dims_size);
    size_t size = 1;
    for (size_t i = 0; i < dims_size; ++i) {
      size_t dim;
      items_read = fread(&(dim), sizeof(size_t), 1, fp);
      size *= dim;
      (*dims)[i] = dim;
    }
    T *data = new T[size];
    items_read = fread(&(data[0]), sizeof(T), size, fp);
    fclose(fp);
    return data;
  }

  // 1D convenience
  template<typename T>
  void WriteArray1D(const std::string &filename, const T *data, size_t dim1) {
    std::vector<size_t> dims;
    dims.push_back(dim1);
    WriteArray(filename, data, dims);
  }

  // 2D convenience
  template<typename T>
  void WriteArray2D(const std::string &filename, const T *data,
      size_t dim1, size_t dim2) {
    std::vector<size_t> dims;
    dims.push_back(dim1);
    dims.push_back(dim2);
    WriteArray(filename, data, dims);
  }

  // 3D convenience
  template<typename T>
  void WriteArray3D(const std::string &filename, const T *data,
      size_t dim1, size_t dim2, size_t dim3) {
    std::vector<size_t> dims;
    dims.push_back(dim1);
    dims.push_back(dim2);
    dims.push_back(dim3);
    WriteArray(filename, data, dims);
  }

}  // namespace ravec

#endif  // _RAVEC_LINALG_H_
