// tools.cpp
//

// TODO: +++ SPLIT INTO DIFFERENT FILES (NAMESPACE ?)

#include "tools.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>


namespace bcp
{

// read an ascii file (column) ans store values in a vector
std::size_t readFileToVector(const std::string& filename, std::vector<double>& v, const int verbose)
{
  double read_value;
  if (verbose > 0) {
    std::cout << "Start reading data file... " << filename << std::endl;
  }
  try {
    std::ifstream mydatafile(filename.data());
    // We check that the file has been found
    if(! mydatafile) {
      throw std::string("Error: readFileToVector >>> cannot open file " + filename);
    }
    // We read the file until the end of file
    while (mydatafile >> read_value) {
      v.push_back(read_value);
    }
    // We close the file
    mydatafile.close();
  } catch (std::string const& error) {
    // We exit if the file was not found
    std::cerr << error << std::endl;
    exit(-1);
  }
  // We print some information on the data
  if (verbose > 0) {
    std::cout << "Range: " << v[0] << " | " << v[v.size()-1] << " Size: " << v.size() << std::endl;
  }
  return v.size();
}

// +++ todo: rewrite/simplify the CSV reading, remove the CSVrow class...
std::istream& safeGetLine(std::istream& is, std::string& str)
{
  str.clear();

  // The characters in the stream are read one-by-one using a std::streambuf.
  // That is faster than reading them one-by-one using the std::istream.
  // Code that uses streambuf this way must be guarded by a sentry object.
  // The sentry object performs various tasks,
  // such as thread synchronization and updating the stream state.

  std::istream::sentry se(is, true);
  std::streambuf* sb = is.rdbuf();

  for(;;) {
    int c = sb->sbumpc();
    switch (c) {
    case '\n':
      return is;
    case '\r':
      if(sb->sgetc() == '\n')
        sb->sbumpc();
      return is;
    case EOF:
      // Also handle the case when the last line has no line ending
      if(str.empty())
        is.setstate(std::ios::eofbit);
      return is;
    default:
      str += (char)c;
    }
  }
}

// class for a row to be read from a CSV file
class CSVRow
{

public:
  CSVRow(const char separator):m_separator(separator) {}

  std::string const& operator[](size_t index) const { return m_data[index]; }
  size_t size() const { return m_data.size(); }
  void readNextRow(std::istream& str);

private:
  std::vector<std::string>  m_data;
  const char m_separator;

};

std::istream& operator>>(std::istream &file, CSVRow &row);
std::istream& safeGetLine(std::istream &file, std::string &line);



void CSVRow::readNextRow(std::istream& str) {
  std::string line;
  // To handle files from an other OS
  safeGetLine(str,line);

  std::stringstream lineStream(line);
  std::string cell;

  m_data.clear();
  while(getline(lineStream,cell,m_separator))
    m_data.push_back(cell);
}


/**
     * Overload of operator>> for our little class CSVRow.
     *
     */
std::istream& operator>>(std::istream& str,CSVRow& data) {
  data.readNextRow(str);
  return str;
}


int readCSVToMatrix(const std::string filename, std::vector<std::vector<double> > &v, const char separator, const int headersRows, const int verbose)
{

  // +++ add checks to detect for instance wrong separator

  CSVRow row(separator);
  std::cout << "Start reading csv file... " << filename << std::endl;
  try {
    std::ifstream mydatafile(filename.data());
    // We check that the file has been found
    if(! mydatafile)
      throw std::string("Error: readCVSToMatrix >>> cannot open file " + filename);

    // skip header
    std::string header;
    for (int i=0; i<headersRows; i++)
      getline(mydatafile,header);

    // read rows
    while (mydatafile >> row)
    {
      if(row.size() > 0) //seems to read 0 size row at the end -_-
      {
        // build row
        std::vector<double> drow;
        for (size_t i=0; i<row.size(); i++)
          drow.push_back(atof(row[i].data()));

        if (verbose > 1)
        {
          for (size_t i=0; i<row.size(); i++)
            std::cout << drow[i] << " ";
          std::cout << std::endl;
        }

        // insert row
        v.push_back(drow);
      }
    }
    mydatafile.close();

  } catch (std::string const& error) {
    // We exit if there was an error
    std::cerr << error << std::endl;
    exit(-1);
  }

  // feedback
  if (verbose > 0)
  {
    std::cout << "Read " << v.size() << " by " << v[0].size() << " matrix" << std::endl;
    std::cout << "First value: " << v[0][0] << " Last value: " << v[v.size()-1][v[v.size()-1].size()-1] << std::endl;
  }

  return (int) v.size();
}



void transpose(std::vector<std::vector<double> > &mat)
{
  if (mat.size() == 0)
    return;

  std::vector<std::vector<double> > tmat(mat[0].size(), std::vector<double>());
  for (size_t i = 0; i < mat.size(); i++)
    for (size_t j = 0; j < mat[i].size(); j++)
      tmat[j].push_back(mat[i][j]);

  mat = tmat;
}


// Write data block (1D) to output file as a single column
void writeDataBlock1D(std::ofstream& file_out, std::string header, const std::vector<double> &datablock)
{
  file_out << header << std::endl;
  for (size_t i = 0; i < datablock.size(); i++)
    file_out << datablock[i] << std::endl;
  file_out << std::endl;
}


// Write data block (2D) to output file as a sequence of single columns
void writeDataBlock2D(std::ofstream& file_out, std::string header, std::vector<std::vector<double> > &datablock)
{
  for (size_t i = 0; i < datablock.size(); i++) {
    file_out << header << " " << i << std::endl;
    for (size_t j = 0; j < datablock[i].size(); j++) {
      file_out << datablock[i][j] << std::endl;
    }
    file_out << std::endl;
  }
}

// read initialisation data from .init file (2 column format) 
void getInitDataFromInitFile(std::string init_file, std::vector<double> & time_data, std::vector<double> & variable_data)
{
    // read 2 column matrix
    std::vector<std::vector<double> > init_data;
    bcp::readCSVToMatrix(init_file, init_data, ' ');
    std::cout << "Read matrix "  << init_data.size() << " by " << init_data[0].size() << std::endl;

    // split the 2 vectors
    for (int i=0; i<init_data.size(); i++)
    {
        if (init_data[i].size() != 2)
        {
            std::cout << "Error reading 2-column init file " << init_file << " Line " << i << " has " << init_data[i].size() << " elements"<< std::endl;
            exit(1);
        }
        time_data.push_back(init_data[i][0]);
        variable_data.push_back(init_data[i][1]);
    }
}

}
//
// tools.cpp ends here
