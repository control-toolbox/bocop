// OCP.cpp
//

#include <OCP.h>

#include <map>
#include <fstream>
#include <iostream>
#include <sstream>

// ///////////////////////////////////////////////////////////////////
// OCP implementation
// ///////////////////////////////////////////////////////////////////


OCP::OCP(void)
{}

OCP::~OCP(void)
{}

void OCP::initialize(void)
{
  preProcessing();
}


int OCP::readDefinitionFile(const std::string& definition_file)
{
  std::ifstream definition_stream(definition_file);
  if (!definition_stream.is_open())
  {
    std::cerr << "ERROR: could not open definition file " << definition_file << std::endl;
    return -1;
  }

  std::string line;
  while (!definition_stream.eof())
  {
    getline(definition_stream, line);
    if (!line.empty() && line[0] != '#')
    {
      std::istringstream pair(line);
      std::string key, value;
      pair >> key;
      pair >> value;
      definition_map.emplace(key,value);
    }
  }
  definition_stream.close();

  return 0;
}


const std::string& OCP::getDefinitionForKey(const std::string& key, const std::string& default_value) const
{
  const auto it = definition_map.find(key);
  if (it == definition_map.end())
  {
    if (default_value == "")
    {
      std::cout << "ERROR: Definition not found for required key \"" << key << "\"" << std::endl;
      exit(1);
    }
    else
    {
      std::cout << "INF0: Using default value " <<  default_value << " for missing key \"" << key << "\"" << std::endl;
      return default_value;
    }

  }
  return it->second;
}


void OCP::setBounds(std::string prefix, const std::size_t dim, std::vector<double> &lower_bounds, std::vector<double> &upper_bounds)
{
  for (std::size_t i = 0;  i < dim; ++i)
  {

    std::stringstream label1, label2;

    // lower bound
    label1 << prefix << "." << i << ".lowerbound";
    auto l1 = definition_map.find(label1.str());
    if (l1 != definition_map.end())
      lower_bounds.push_back(stod(definition_map.at(label1.str())));
    else
    {
      lower_bounds.push_back(std::numeric_limits<double>::lowest());
      freebounds << label1.str() << " ";
    }

    // upper bound
    label2 << prefix << "." << i << ".upperbound";
    auto l2 = definition_map.find(label2.str());
    if (l2 != definition_map.end())
      upper_bounds.push_back(stod(definition_map.at(label2.str())));
    else
    {
      upper_bounds.push_back(std::numeric_limits<double>::max());
      freebounds << label2.str() << " ";
    }
  }
}


void OCP::load(const std::string& problem_file)
{
  // read definition file and fill map
  int ok = readDefinitionFile(problem_file);

  if (ok == 0)
  {
    // set dimensions
    state_size= std::stoi(getDefinitionForKey("dim.state"));
    control_size = std::stoi(getDefinitionForKey("dim.control"));
    boundary_conditions_size = std::stoi(getDefinitionForKey("dim.boundaryconditions"));
    path_constraints_size = std::stoi(getDefinitionForKey("dim.pathconstraints"));
    parameters_size = std::stoi(getDefinitionForKey("dim.parameters"));
    constants_size = std::stoi(getDefinitionForKey("dim.constants"));
    // set time interval. NB normalized to [0,1] if final time is free
    initial_time = std::stod(getDefinitionForKey("initial.time"));
    //final_time = std::stod(getDefinitionForKey("final.time"));
    // detect free final time case
    std::string final_time_type = getDefinitionForKey("final.time");
    if (final_time_type.find("free") != std::string::npos)
    {
      free_final_time = true;
      final_time = 1.0;
    }
    else
    {
      free_final_time = false;
      final_time = stod(final_time_type);
    }

    // set constants
    for (size_t i = 0; i < constants_size; ++i)
    {
      std::stringstream label;
      label << "constant." << i;
      constants.push_back(std::stod(getDefinitionForKey(label.str())));
    }

    // set bounds for variables and constraints
    setBounds("boundarycond", boundary_conditions_size, boundary_conditions_lower_bounds, boundary_conditions_upper_bounds);
    setBounds("pathconstraint", path_constraints_size, path_constraints_lower_bounds, path_constraints_upper_bounds);
    setBounds("state", state_size, state_lower_bounds, state_upper_bounds);
    setBounds("control", control_size, control_lower_bounds, control_upper_bounds);
    setBounds("parameter", parameters_size, param_lower_bounds, param_upper_bounds);

    if (freebounds.rdbuf()->in_avail() != 0)
      std::cout << "INFO: using free bounds for variables: " << freebounds.str() << std::endl;
  }

}


void OCP::save(std::ofstream& file_out)
{
  file_out << "########################################################################" << std::endl;
  file_out << "# COPY OF DEFINITION FILE " << std::endl;
  file_out << "########################################################################" << std::endl;
  for (const auto& it : definition_map) {
    file_out << it.first << " " << it.second << std::endl;
  }
  file_out << std::endl;
}



//
// OCP.cpp ends here
