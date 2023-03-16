// tools.h
//

// +++ todo: split into sub files tools_fileIO, tools_interpolation to be included in main .h/.cpp

#pragma once

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>


// regroup other tools_ headers here so we only have to include tools.h
#include <bufferAdaptor.h>
#include <tools_interpolation.h>

namespace bcp
{

// read data files
std::size_t readFileToVector(const std::string& filename, std::vector<double>& v, const int verbose = 1);
double normalizedTimeInterpolation(const double, const std::vector<double>&);
void writeDataBlock1D(std::ofstream& file_out, std::string header, const std::vector<double> &datablock);
void writeDataBlock2D(std::ofstream& file_out, std::string header, std::vector<std::vector<double> > &datablock);
int readCSVToMatrix(const std::string filename, std::vector<std::vector<double> > &v, const char separator, const int headersRows=0, const int verbose=0);
void transpose(std::vector<std::vector<double> > &mat);
void getInitDataFromInitFile(std::string init_file, std::vector<double> & time_data, std::vector<double> & variable_data);

}




//
// tools.h ends here
