// dODE.h
//

#pragma once

#include <OCP.h>
#include <solution.h>

/** ***************************************************************************
* \class dODERK
* \brief dODERK implements the discretization of the ODE from OCP\n
* It uses a generalized Runge Kutta formula defined in the Butcher format
* to build the set of discrete constraints corresponding to the continuous ODE
* ****************************************************************************/
class dODERK
{
public:

  void initialize();

  void setRKcoeffs(const std::string RKmethod);
  void setTimeGrids(const double t0, const double tf, const std::size_t discretisation_steps, double &time_step, std::vector<double> &time_step_grid, std::vector<double> &time_stage_grid);

  void setInitialParam(OCP *ocp, std::vector<double> &starting_point, std::vector<double> &variables_lower_bounds, std::vector<double> &variables_upper_bounds);
  void setInitialState(const std::vector<double> time_steps, OCP *ocp, std::vector<double> &starting_point, std::vector<double> &variables_lower_bounds, std::vector<double> &variables_upper_bounds);
  void setInitialControl(const std::vector<double> discretisation_stages, OCP *ocp, dODERK *rk, std::vector<double> &starting_point, std::vector<double> &variables_lower_bounds, std::vector<double> &variables_upper_bounds);
  void setRKStageVars(size_t discretisation_steps, OCP *ocp, std::vector<double> &starting_point, std::vector<double> &variables_lower_bounds, std::vector<double> &variables_upper_bounds);

  // +++ merge as setConstraints
  void setBoundaryConditionsBounds(OCP *ocp, std::vector<double> &constraints_lower_bounds, std::vector<double> &constraints_upper_bounds);
  void setDiscretisedConstraintsBounds(size_t discretisation_steps, OCP *ocp, std::vector<double> &constraints_lower_bounds, std::vector<double> &constraints_upper_bounds);

  // get split variables and multipliers
  void getVariables(const double* x, const double* mult_lowerbounds, const double* mult_upperbounds, OCP *ocp,
                    std::vector<std::vector<double> >& state, std::vector<std::vector<double> >& control, std::vector<double>& parameter, 
                    std::vector<std::vector<double> >& state_lowerbound_multiplier, std::vector<std::vector<double> >& control_lowerbound_multiplier, std::vector<double>& parameter_lowerbound_multiplier,
                    std::vector<std::vector<double> >& state_upperbound_multiplier, std::vector<std::vector<double> >& control_upperbound_multiplier, std::vector<double>& parameter_upperbound_multiplier);
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

  std::size_t variables_offset_state;
  std::size_t variables_offset_control;
  std::size_t variables_offset_param;

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
// dODERK.h ends here
