#pragma once

#include <cstdlib>
#include <iostream>
#include <vector>


namespace bcp
{

// interpolations
void interpolatelValuesOnGrid(const std::vector<double> x_values, const std::vector<double> x_data, const std::vector<double> y_data, std::vector<double>& y_values);

template<typename Variable> std::size_t locateInArray(const Variable value, const double *data, const std::size_t data_size, const int verbose = 1);
template<typename Variable> std::size_t locate(const Variable x_value, const std::vector<double> &x_data, const int verbose = 1) { return locateInArray(x_value,x_data.data(),x_data.size(),verbose); }
double interpolation1Dlinear(const double x_value, const std::vector<double> &x_data, const std::vector<double> &y_data, const std::size_t set_data_size=0, const int verbose=0);
template<typename Variable> Variable interpolation2Dbilinear(const Variable x, const Variable y, const std::vector<double>& x_data, const std::vector<double>& y_data, const std::vector<std::vector<double> > &z_data, const int verbose=0);


// definitions for template functions

// return the index in [0, data.size - 2] such that data[index] <= value < data[index+1]
// out of bounds values are treated by projection ie index = 0 or data.size - 2
// note: use of upper_bounds from stl does not seem faster ...
template<typename Variable> std::size_t locateInArray(const Variable value, const double *data, const std::size_t data_size, const int verbose)
{

  std::size_t index = 0;

  // Test for out_of_bounds values
  double epsilon = 1e-6;
  if (value < data[0] * (1e0 - epsilon))
  {
    index = 0;
    if (verbose > 0)
      std::cout << "WARNING: locate index is out of bounds: " << value << " lower than first element in grid " << data[0] << std::endl;
  }
  else if (value > data[data_size-1] * (1e0 + epsilon))
  {
    index = data_size - 2;
    if (verbose > 0)
      std::cout << "WARNING: locate index is out of bounds: " << value << " greater than last element (" << data_size-1<< ") in grid " << data[data_size-1] << std::endl;
  }
  else
    while (index < data_size - 2 && value >= data[index+1])
      index++;

  return index;
}


// 2D interpolation
template <typename Variable> Variable interpolation2Dbilinear(const Variable x, const Variable y, const std::vector<double>& x_data, const std::vector<double>& y_data,
                       const std::vector<std::vector<double> > &z_data, const int verbose)
{

  Variable z = 0e0;

  // check sizes
  size_t dimX = x_data.size();
  size_t dimY = y_data.size();
  size_t dimZ1 = z_data.size();
  size_t dimZ2 = z_data[0].size();
  if (dimZ1 != dimX || dimZ2 != dimY)
  {
    std::cout << "ERROR: interpolation2Dbilinear >>> dimensions mismatch (X,Y): " << dimX << " "  << dimY << " with Z: " << dimZ1 << " " << dimZ2 << std::endl;
    exit(1);
  }

  // locate row/column index in grid
  // NB. correct derivatives require retaping !
  size_t i = locate(x, x_data, verbose);
  size_t j = locate(y, y_data, verbose);
  double x1 = x_data[i];
  double x2 = x_data[i+1];
  double y1 = y_data[j];
  double y2 = y_data[j+1];
  double f11 = z_data[i][j];
  double f12 = z_data[i][j+1];
  double f21 = z_data[i+1][j];
  double f22 = z_data[i+1][j+1];

  //+++ NB: out of bounds values for x,y will lead to linear extrapolation...
  z = f11*(x2-x)*(y2-y) + f21*(x-x1)*(y2-y) + f12*(x2-x)*(y-y1) + f22*(x-x1)*(y-y1);
  z /= (x2-x1)*(y2-y1);

  return z;
}


// NB. cannot use .data() on a AD<double> vector... Use locateInArray instead.



}
