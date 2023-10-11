// dODE.h
//

#pragma once

#include <OCP.h>


/** ***************************************************************************
* \class dODE
* \brief dODE implements the discretization of the ODE from OCP\n
* It uses a generalized Runge Kutta formula defined in the Butcher format
* to build the set of discrete constraints corresponding to the continuous ODE
* ****************************************************************************/
class dODE
{
public:

  void initialize();
  
  void setRKcoeffs(const std::string RKmethod);
  void setTimeGrids(const double t0, const double tf, const std::size_t discretisation_steps, double &time_step, std::vector<double> &time_step_grid, std::vector<double> &time_stage_grid);

  size_t setInitialParam(OCP *ocp, std::vector<double> &starting_point, std::vector<double> &variables_lower_bounds, std::vector<double> &variables_upper_bounds);
  void setRKStageVars(size_t discretisation_steps, OCP *ocp, std::vector<double> &starting_point, std::vector<double> &variables_lower_bounds, std::vector<double> &variables_upper_bounds);

  // +++ merge as setConstraints
  void setBoundaryConditionsBounds(OCP *ocp, std::vector<double> &constraints_lower_bounds, std::vector<double> &constraints_upper_bounds);
  void setDiscretisedConstraintsBounds(size_t discretisation_steps, OCP *ocp, std::vector<double> &constraints_lower_bounds, std::vector<double> &constraints_upper_bounds);

  void getParam(const double* x, size_t param_offset, size_t dimParam, std::vector<double>& parameter);
  void getMultipliers(const double* lambda, std::vector<double>& boundaryCondMultiplier, std::vector<std::vector<double> >&  pathConstrMultiplier, std::vector<std::vector<double> >&  adjointState);
  void getConstraints(const double* g, std::vector<double>& boundaryConditions, std::vector<std::vector<double> >&  pathConstraints, std::vector<std::vector<double> >& dynEquations);


  //void getBoundaryConditions
  //void getPathConstraints

  // getters
  const std::vector<std::vector<double>>& butcherA(void) { return butcher_a; };
  const std::vector<double>& butcherB(void) { return butcher_b; } ;
  const std::vector<double>& butcherC(void) { return butcher_c; } ;
  const std::size_t RKStages(void) { return rk_stages; };

public:

  // coefficients for the generalized Runge Kutta method (using butcher notations)
  // c1   a11 ... a1s
  // .    .       .
  // cs   as1 ... ass
  //       b1 ... bs
  std::size_t rk_stages;
  std::vector<double> butcher_b;
  std::vector<double> butcher_c;
  std::vector<std::vector<double>> butcher_a;

};


//
// dODE.h ends here
