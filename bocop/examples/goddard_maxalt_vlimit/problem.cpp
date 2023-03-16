
// +++DRAFT+++ This class implements the OCP functions
// It derives from the generic class bocop3OCPBase
// OCP functions are defined with templates since they will be called
// from both the NLP solver (double arguments) and AD tool (ad_double arguments)
//#pragma once

#include <OCP.h>
// ///////////////////////////////////////////////////////////////////
template <typename Variable>
void OCP::finalCost(double initial_time, double final_time, const Variable *initial_state, const Variable *final_state, const Variable *parameters, const double *constants, Variable &final_cost)
{
  // MAXIMIZE FINAL ALTITUDE
  final_cost = - final_state[0];
}

template <typename Variable>
void OCP::dynamics(double time, const Variable *state, const Variable *control, const Variable *parameters, const double *constants, Variable *state_dynamics)
{
  // dr/dt = v
  // dv/dt = (Thrust(u) - Drag(r,v)) / m - grav(r)
  // dm/dt = -b Tmax |u|
  double Tmax = constants[0];
  double Cd = constants[1];
  double beta = constants[2];
  double b = constants[3];

  Variable r = state[0];
  Variable v = state[1];
  Variable m = state[2];
  Variable u = control[0];

  state_dynamics[0] = v;
  state_dynamics[1] = (u *  Tmax - Cd * v * v * exp(-beta*(fabs(r)-1e0))) / m - 1e0 / (r*r);
  state_dynamics[2] = - b * Tmax * u;

  // free final time: rescale dynamics
  Variable tf = parameters[0];
  for (size_t i=0; i<3; i++)
    state_dynamics[i] *= tf;
}

template <typename Variable>
void OCP::boundaryConditions(double initial_time, double final_time, const Variable *initial_state, const Variable *final_state, const Variable *parameters, const double *constants, Variable *boundary_conditions)
{
  // r0 = 1    v0 = 0   m0 = 1
  boundary_conditions[0] = initial_state[0];
  boundary_conditions[1] = initial_state[1];
  boundary_conditions[2] = initial_state[2];
}

template <typename Variable>
void OCP::pathConstraints(double time, const Variable *state, const Variable *control, const Variable *parameters, const double *constants, Variable *path_constraints)
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
