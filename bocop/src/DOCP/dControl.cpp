// dControl.cpp
//

#include <dControl.h>


// each function fill will increment the offset for the next call
// +++ mutualise a bit more with dState ?
void dControl::setInitialControl(const std::vector<double> time_stages, OCP *ocp, dODE *rk, std::vector<double> &starting_point, std::vector<double> &variables_lower_bounds, std::vector<double> &variables_upper_bounds)
{
  size_t discretisation_stages = time_stages.size();

  // compute initial values for control
  std::vector<std::vector<double> > control_init(ocp->controlSize(), std::vector<double>(discretisation_stages, 666));
  for (std::size_t k = 0; k < ocp->controlSize(); k++)
  {
    // get initialisation type
    std::stringstream label;
    label << "control." << k << ".init";
    std::string control_init_type = ocp->getDefinitionForKey(label.str());

    // set initial values
    if (control_init_type.find(".init") != std::string::npos)
    {
      // read .init file and interpolate values over time stages
      std::vector<double> time_data, variable_data;
      bcp::getInitDataFromInitFile(control_init_type, time_data, variable_data);
      bcp::interpolatelValuesOnGrid(time_stages, time_data, variable_data, control_init[k]);
    }
    else
      // constant initialisation from .def file
      for (std::size_t i = 0; i < discretisation_stages; i++)
          control_init[k][i] = stod(control_init_type);
  }

  // set control variables bounds and initialisation
  for (size_t i = 0; i < discretisation_stages; i++)
      for (size_t k = 0; k < ocp->controlSize(); k++)
      {
        variables_lower_bounds.push_back(ocp->controlLowerBounds()[k]);
        variables_upper_bounds.push_back(ocp->controlUpperBounds()[k]);
        starting_point.push_back(control_init[k][i]);
      }
}

// +++ this one is not necessarily compatible with controlAtStage that takes a template C... 
void dControl::getControl(const double* x, size_t control_offset, size_t dimSteps, size_t dimStages, size_t dimControl, std::vector<std::vector<double> >& control)
{
  size_t index_x = control_offset;
  size_t index_u = 0;
  for (size_t i = 0; i < dimSteps; i++)
    for (size_t l = 0; l < dimStages; l++)
    {
      for (size_t j = 0; j < dimControl; j++)
        control[j][index_u] = x[index_x++];
      index_u++;
    }
}

//
// dControl.cpp ends here
