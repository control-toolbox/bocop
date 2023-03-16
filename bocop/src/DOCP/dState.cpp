// dState.cpp
//

#include <dState.h>


// each function fill will increment the offset for the next call
// +++ maybe mutualise a bit with setControl and setParam via aux functions ?
void dState::setInitialState(const std::vector<double> time_steps, OCP *ocp, std::vector<double> &starting_point, std::vector<double> &variables_lower_bounds, std::vector<double> &variables_upper_bounds)
{

  size_t discretisation_steps = time_steps.size() - 1;

  // compute initial values for state
  std::vector<std::vector<double> > state_init(ocp->stateSize(), std::vector<double>(discretisation_steps+1,666));
  for (std::size_t k = 0; k < ocp->stateSize(); k++)
  {
    // get initialisation type
    std::stringstream label;
    label << "state." << k << ".init";
    std::string state_init_type = ocp->getDefinitionForKey(label.str());

    // set initial values
    if (state_init_type.find(".init") != std::string::npos)
    {
      // read .init file and interpolate values over time steps
      std::vector<double> time_data, variable_data;
      bcp::getInitDataFromInitFile(state_init_type, time_data, variable_data);
      bcp::interpolatelValuesOnGrid(time_steps, time_data, variable_data, state_init[k]);
    }
    else
      // constant initialisation from .def file
      for (std::size_t i = 0; i < discretisation_steps+1; i++)
        state_init[k][i] = stod(state_init_type);


    // DEBUG
    /*for (size_t i=0;i<time_steps.size();i++)
      std::cout << state_init[k][i] << " ";
    std::cout << std::endl;
    exit(1);*/
  }

  // set state variables bounds and initialisation
  for (std::size_t i = 0; i < discretisation_steps+1; i++)
    for (std::size_t k = 0; k < ocp->stateSize(); k++)
    {
      variables_lower_bounds.push_back(ocp->stateLowerBounds()[k]);
      variables_upper_bounds.push_back(ocp->stateUpperBounds()[k]);
      starting_point.push_back(state_init[k][i]);
    }
}


void dState::getState(const double* x, size_t state_offset, size_t dimSteps, size_t dimState, std::vector<std::vector<double> >& state)
{
  size_t index_x = state_offset;
  for (size_t i=0; i < dimSteps+1; i++)
    for (size_t j = 0; j < dimState; j++)
      state[j][i] = x[index_x++];
}

//
// dState.cpp ends here
