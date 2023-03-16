// dControl.h
//

#pragma once

#include <OCP.h>
#include <dODE.h>

/** ***************************************************************************
* \class dControl
* \brief dControl implements the control variables handling in dOCP
* ****************************************************************************/
class dControl
{
public:

  void setInitialControl(const std::vector<double> discretisation_stages, OCP *ocp, dODE *rk, std::vector<double> &starting_point, std::vector<double> &variables_lower_bounds, std::vector<double> &variables_upper_bounds);

  void getControl(const double* x, size_t control_offset, size_t dimSteps, size_t dimStages, size_t dimControl, std::vector<std::vector<double> >& control);

};


//
// dControl.h ends here
