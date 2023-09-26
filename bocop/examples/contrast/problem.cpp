


// +++DRAFT+++ This class implements the OCP functions
// It derives from the generic class bocop3OCPBase
// OCP functions are defined with templates since they will be called
// from both the NLP solver (double arguments) and AD tool (ad_double arguments)
//#pragma once

#include <OCP.h>

// ///////////////////////////////////////////////////////////////////


template <typename Variable>
inline void OCP::finalCost(double initial_time, double final_time, const Variable *initial_state, const Variable *final_state, const Variable *parameters, const double *constants, Variable &final_cost)
{
  Variable norm2 = pow(final_state[2],2) + pow(final_state[3],2);
  final_cost = - sqrt(norm2) + final_state[4];
}

template <typename Variable>
inline void OCP::dynamics(double time, const Variable *state, const Variable *control, const Variable *parameters, const double *constants, Variable *state_dynamics)
{

  // constants
  double pi = 3.141592653589793e0;
  double omegamax = constants[0];
  double T11 = constants[1];
  double T12 = constants[2];
  double T21 = constants[3];
  double T22 = constants[4];
  double Tmin = constants[5];
  double tmincoeff = constants[6];
  double regcoeff = constants[7];
  
  double biggamma1 = 1e0 / (T12 * omegamax);
  double smallgamma1 = 1e0 / (T11 * omegamax);
  double biggamma2 = 1e0 / (T22 * omegamax);
  double smallgamma2 = 1e0 / (T21 * omegamax);
  
  // variables
  Variable y1 = state[0];
  Variable z1 = state[1];
  Variable y2 = state[2];
  Variable z2 = state[3];
  Variable ux = control[0];

  // dynamics for 1st and 2nd spin
  state_dynamics[0] = - biggamma1 * y1 - ux * z1;
  state_dynamics[1] = smallgamma1 * (1e0 - z1) + ux * y1;
  state_dynamics[2] = - biggamma2 * y2 - ux * z2;
  state_dynamics[3] = smallgamma2 * (1e0 - z2) + ux * y2;	

  //quadratic regularization
  state_dynamics[4] = pow(10e0,-regcoeff) * ux * ux;

  //manual time normalization
  for (int i=0;i<dim_state;i++) state_dynamics[i] *= Tmin * tmincoeff * 2.0 * pi;

}

template <typename Variable>
inline void OCP::boundaryConditions(double initial_time, double final_time, const Variable *initial_state, const Variable *final_state, const Variable *parameters, const double *constants, Variable *boundary_conditions)
{
  //1st spin: initial and final conditions
  boundary_conditions[0] = initial_state[0];
  boundary_conditions[1] = initial_state[1];
  boundary_conditions[2] = final_state[0];
  boundary_conditions[3] = final_state[1];

  //2nd spin: initial conditions
  boundary_conditions[4] = initial_state[2];
  boundary_conditions[5] = initial_state[3];

  //regularisation
  boundary_conditions[6] = initial_state[4];
}

template <typename Variable>
inline void OCP::pathConstraints(double time, const Variable *state, const Variable *control, const Variable *parameters, const double *constants, Variable *path_constraints)
{
}

void OCP::preProcessing()
{}

// ///////////////////////////////////////////////////////////////////
// explicit template instanciation for template functions, with double and double_ad 
// +++ could be in an included separate file ? 
// but needs to be done for aux functions too ? APPARENTLY NOT !
template void OCP::finalCost<double>(double initial_time, double final_time, const double *initial_state, const double *final_state, const double *parameters, const double *constants, double &final_cost);
template void OCP::dynamics<double>(double time, const double *state, const double *control, const double *parameters, const double *constants, double *state_dynamics);
template void OCP::boundaryConditions<double>(double initial_time, double final_time, const double *initial_state, const double *final_state, const double *parameters, const double *constants, double *boundary_conditions);
template void OCP::pathConstraints<double>(double time, const double *state, const double *control, const double *parameters, const double *constants, double *path_constraints);

template void OCP::finalCost<double_ad>(double initial_time, double final_time, const double_ad *initial_state, const double_ad *final_state, const double_ad *parameters, const double *constants, double_ad &final_cost);
template void OCP::dynamics<double_ad>(double time, const double_ad *state, const double_ad *control, const double_ad *parameters, const double *constants, double_ad *state_dynamics);
template void OCP::boundaryConditions<double_ad>(double initial_time, double final_time, const double_ad *initial_state, const double_ad *final_state, const double_ad *parameters, const double *constants, double_ad *boundary_conditions);
template void OCP::pathConstraints<double_ad>(double time, const double_ad *state, const double_ad *control, const double_ad *parameters, const double *constants, double_ad *path_constraints);
