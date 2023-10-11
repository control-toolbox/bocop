
// +++DRAFT+++ This class implements the OCP functions
// It derives from the generic class bocop3OCPBase
// OCP functions are defined with templates since they will be called
// from both the NLP solver (double arguments) and AD tool (ad_double arguments)
//#pragma once

#include <OCP.h>
// ///////////////////////////////////////////////////////////////////


// aux functions

// FUNCTION FOR GODDARD DRAG
// drag = 310 v^2 exp (-500(r-1))
// Arguments:
// r: radius
// v: velocity
template <typename Variable>
inline Variable drag(const Variable r, const Variable v, double A, double k, double r0)
{
  Variable drag = A * v * v * exp(-k*(fabs(r)-r0));
  return drag;
}

// FUNCTION FOR GRAVITY
// g = 1 / r^2
// Arguments:
// r: radius
template <typename Variable>
inline Variable grav(const Variable r)
{
  Variable grav = 1e0 / r / r;
  return grav;
}

// FUNCTION FOR THRUST (GODDARD)
// Variable = u * Tmax
// Arguments:
// r: radius
template <typename Variable>
inline Variable thrust(const Variable u, double Tmax)
{
  Variable thrust = u * Tmax;
  return thrust;
}



// ///////////////////////////////////////////////////////////////////


template <typename Variable>
void OCP::finalCost(double initial_time, Variable final_time, const Variable *initial_state, const Variable *final_state, const Variable *parameters, const double *constants, Variable &final_cost)
{
  // CRITERION FOR GODDARD PROBLEM
  // MAXIMIZE FINAL MASS
  final_cost = -final_state[2];
}

template <typename Variable>
void OCP::dynamics(Variable time, const Variable *state, const Variable *control, const Variable *parameters, const double *constants, Variable *state_dynamics)
{
  // DYNAMICS FOR GODDARD PROBLEM
  // dr/dt = v
  // dv/dt = (Thrust(u) - Drag(r,v)) / m - grav(r)
  // dm/dt = -b*|u|

  double Tmax = constants[0];
  double A = constants[1];
  double k = constants[2];
  double r0 = constants[3];
  double b = constants[4];

  Variable r = state[0];
  Variable v = state[1];
  Variable m = state[2];

  state_dynamics[0] = v;
  state_dynamics[1] = (thrust(control[0],Tmax) - drag(r,v,A,k,r0)) / m - grav(r);
  state_dynamics[2] = - b * control[0];
}

template <typename Variable>
void OCP::boundaryConditions(double initial_time, Variable final_time, const Variable *initial_state, const Variable *final_state, const Variable *parameters, const double *constants, Variable *boundary_conditions)
{
  // INITIAL CONDITIONS FOR GODDARD PROBLEM
  // r0 = 1    v0 = 0   m0 = 1
  // MODELED AS 1 <= r0 <= 1, etc
  boundary_conditions[0] = initial_state[0];
  boundary_conditions[1] = initial_state[1];
  boundary_conditions[2] = initial_state[2];

  // FINAL CONDITIONS FOR GODDARD PROBLEM
    // rf >= 1.01   MODELED AS   1.01 <= rf
  boundary_conditions[3] = final_state[0];
}

template <typename Variable>
void OCP::pathConstraints(Variable time, const Variable *state, const Variable *control, const Variable *parameters, const double *constants, Variable *path_constraints)
{
  // CONSTRAINT ON MAX DRAG FOR GODDARD PROBLEM
  // Drag <= C ie Drag - C <= 0

  double A = constants[1];
  double k = constants[2];
  double r0 = constants[3];
  double C = constants[5];

  Variable r = state[0];
  Variable v = state[1];

  path_constraints[0] = drag(r,v,A,k,r0) - C;
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

template void OCP::finalCost<double_ad>(double initial_time, double_ad final_time, const double_ad *initial_state, const double_ad *final_state, const double_ad *parameters, const double *constants, double_ad &final_cost);
template void OCP::dynamics<double_ad>(double_ad time, const double_ad *state, const double_ad *control, const double_ad *parameters, const double *constants, double_ad *state_dynamics);
template void OCP::boundaryConditions<double_ad>(double initial_time, double_ad final_time, const double_ad *initial_state, const double_ad *final_state, const double_ad *parameters, const double *constants, double_ad *boundary_conditions);
template void OCP::pathConstraints<double_ad>(double_ad time, const double_ad *state, const double_ad *control, const double_ad *parameters, const double *constants, double_ad *path_constraints);
