


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
  // see dynamics for the running cost
  final_cost = final_state[stateSize()-1];
}

template <typename Variable>
inline void OCP::dynamics(double time, const Variable *state, const Variable *control, const Variable *parameters, const double *constants, Variable *state_dynamics)
{
  // constants
	double alpha = constants[0];
	double beta = constants[1];
	double gamma = constants[2];

	// INTEGRATOR X(n) = U
	for(int i=0;i<stateSize()-2;i++)
		state_dynamics[i] = state[i+1];

	state_dynamics[stateSize() - 2] = control[0];

	// CRITERION min int (alpha x + beta x^2 + gamma u^2 )
	state_dynamics[stateSize() - 1] = alpha*state[0] + beta*state[0]*state[0] + gamma*control[0]*control[0];

}

template <typename Variable>
inline void OCP::boundaryConditions(double initial_time, double final_time, const Variable *initial_state, const Variable *final_state, const Variable *parameters, const double *constants, Variable *boundary_conditions)
{
	// INITIAL CONDITIONS FOR STATECONSTRAINTS3 PROBLEM
	// X = (1,0, ... ,0) + CRIT
	for(int i=0;i<stateSize();i++)
		boundary_conditions[i] = initial_state[i];

	// FINAL CONDITIONS FOR STATECONSTRAINTS3 PROBLEM 
	// X = 0
	for(int i=0;i<stateSize()-1;i++)
		boundary_conditions[stateSize() + i] = final_state[i];
}

template <typename Variable>
inline void OCP::pathConstraints(double time, const Variable *state, const Variable *control, const Variable *parameters, const double *constants, Variable *path_constraints)
{
  // NB. state constraint X_0 >= 0 is set by a variable bound instead of a nonlinear constraint here
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
