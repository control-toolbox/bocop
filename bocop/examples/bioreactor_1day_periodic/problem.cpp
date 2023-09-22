


// +++DRAFT+++ This class implements the OCP functions
// It derives from the generic class bocop3OCPBase
// OCP functions are defined with templates since they will be called
// from both the NLP solver (double arguments) and AD tool (ad_double arguments)
//#pragma once

#include <OCP.h>

// ///////////////////////////////////////////////////////////////////
// AUX FUNCTIONS

// BIOMASS GROWTH
// Arguments:
// s: substrate
// mu2m: parameter
// K: parameter
template<typename Variable> Variable growth(const Variable s, const double mu2m, const double K)
{
  // MONOD
	Variable growth = mu2m * s / (s+K);
  return growth;
}

// ALGAE GROWTH
// Arguments:
// mubar: day growth
// time: current time
// halfperiod: day duration
template<typename Variable> Variable daynightgrowth(const double mubar, const Variable time, const double halfperiod)
{
	// light model: max^2 (0,sin) * mubar
	// DAY/NIGHT CYCLE: [0,2 halfperiod] rescaled to [0,2pi]
	double pi = 3.141592653589793;
	Variable days = time / (halfperiod*2e0);
  Variable tau = (days - CppAD::Integer(days)) * 2e0*pi;
	Variable zero = 0e0;
  Variable	mu = pow(fmax(zero,sin(tau)),2) * mubar;
	return mu;
}


template <typename Variable>
inline void OCP::finalCost(double initial_time, double final_time, const Variable *initial_state, const Variable *final_state, const Variable *parameters, const double *constants, Variable &final_cost)
{
  // maximize methane production
  final_cost = - final_state[3];
}

template <typename Variable>
inline void OCP::dynamics(double time, const Variable *state, const Variable *control, const Variable *parameters, const double *constants, Variable *state_dynamics)
{
  /* METHANE PROBLEM
  y' = mu y / (1+y) - (r+u)y
  s' = -mu2(s) x + u beta (gamma y-s)
  x' = (mu2(s) - u beta)x
  J' = - mu2*x / (beta+c)
  with
  mu2 according to growth model
  mu according to light model
  time scale is [0,10] for 24h (day then night)
  (changing this would require adjusting some other constants)*/

  // constants
  double mubar = constants[0];
  double r = constants[1];
  double beta = constants[2];
  double gamma = constants[3];
  double mu2m = constants[4];
  double Ks = constants[5];
  double halfperiod = constants[6];
  double c = constants[7];

//Variable beta = optimvars[0];

  // variables
  Variable y = state[0];
  Variable s = state[1];
  Variable x = state[2];
  Variable u = control[0];

  //biomass growth
  Variable mu2 = growth(s,mu2m,Ks);

  //algae growth
  Variable mu = daynightgrowth(mubar,time,halfperiod);

  // dynamics
  state_dynamics[0] = mu*y/(1+y) - (r+u)*y;
  state_dynamics[1] = -mu2*x + u*beta*(y*gamma-s);
  state_dynamics[2] = (mu2-u*beta)*x;
  state_dynamics[3] = mu2*x / (beta+c);
}

template <typename Variable>
inline void OCP::boundaryConditions(double initial_time, double final_time, const Variable *initial_state, const Variable *final_state, const Variable *parameters, const double *constants, Variable *boundary_conditions)
{
  // initial conditions
  boundary_conditions[0] =  initial_state[0];
  boundary_conditions[1] = initial_state[1];
  boundary_conditions[2] = initial_state[2];
  boundary_conditions[3] = initial_state[3];

  // periodicity
  boundary_conditions[4] = final_state[0] - initial_state[0];
  boundary_conditions[5] = final_state[1] - initial_state[1];
  boundary_conditions[6] = final_state[2] - initial_state[2];
}

template <typename Variable>
inline void OCP::pathConstraints(double time, const Variable *state, const Variable *control, const Variable *parameters, const double *constants, Variable *path_constraints)
{
  // dummy constraint to visualize mu for light model
  double mubar = constants[0];
  double halfperiod = constants[6];
  Variable mu = daynightgrowth(mubar,time,halfperiod);
  path_constraints[0] = mu;
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
