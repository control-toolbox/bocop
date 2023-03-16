// dState.h
//

#pragma once

#include <OCP.h>

/** ***************************************************************************
* \class dState
* \brief dState implements the state variables handling in dOCP
* ****************************************************************************/
class dState
{
public:

  void setInitialState(const std::vector<double> time_steps, OCP *ocp, std::vector<double> &starting_point, std::vector<double> &variables_lower_bounds, std::vector<double> &variables_upper_bounds);

  void getState(const double* x, size_t state_offset, size_t discretisation_steps, size_t stateSize, std::vector<std::vector<double> >& state);

};


//
// dState.h ends here
