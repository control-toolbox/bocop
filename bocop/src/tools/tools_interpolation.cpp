#include "tools_interpolation.h"

namespace bcp
{


void interpolatelValuesOnGrid(const std::vector<double> x_values, const std::vector<double> x_data, const std::vector<double> y_data, std::vector<double>& y_values)
{
  // check dims+++ and debug
  //std::cout


  int verbose = 0;
  for (size_t i=0; i<x_values.size(); i++)
    y_values[i] = interpolation1Dlinear(x_values[i], x_data, y_data, x_data.size(), verbose);

}


// 1D interpolation
// +++ TODO fix potential bug when passing verbose but not set_data_size: verboise is then taken as set_data_size -_- !!!
double interpolation1Dlinear(const double x_value, const std::vector<double> &x_data, const std::vector<double> &y_data, const std::size_t set_data_size, const int verbose)
{
	
  // check for equal vectors size when size is set to auto
  if (set_data_size == 0 && x_data.size() != y_data.size())
  {
    std::cerr << "Error: interpolation between vectors with different size: " << x_data.size() << " and " << y_data.size() << std::endl;
    exit(-66);
  }

  // set x data size
  size_t data_size;
  if (set_data_size == 0)
    data_size = x_data.size();
  else
    data_size = set_data_size;

  // locate position of x_value in x_data
  int index = locateInArray(x_value, x_data.data(), data_size);

  // out of bounds; take value of lower / upper bound
  double slope, y_value;
  if (index == -1)
  {
    if (verbose > 0)
      std::cout << "Warning: the x_value you specified for interpolation is below upper bound of x_data.\nx_value = " << x_value << "<" << x_data[0] << " = x_data lower bound." << std::endl;
    y_value = y_data[0];
    slope = 0e0;
  }
  else if (index == -2)
  {
    if (verbose > 0)
      std::cout << "Warning: the x_value you specified for interpolation is above upper bound of x_data.\nx_value = " << x_value << ">" << x_data[data_size-1] << " = x_data upper bound." << std::endl;
    y_value = y_data[data_size-1];
    slope = 0e0;
  }
  else
  {
    slope = (y_data[index+1] - y_data[index]) / (x_data[index+1] - x_data[index]);
    y_value = y_data[index] + slope * (x_value - x_data[index]);
  }

  return y_value;
}


}
