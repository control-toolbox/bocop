


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
  final_cost = final_state[stateSize()-1];
}

template <typename Variable>
inline void OCP::dynamics(double time, const Variable *state, const Variable *control, const Variable *parameters, const double *constants, Variable *state_dynamics)
{
  // DYNAMICS FOR HEAT PROBLEM
   
  // Spatial step
  double h = 1.0/(stateSize()-1);
  double h2 = h * h;
   
  // constants
  double c0 = constants[0];
  double c1 = constants[1];
  double a = constants[2]; //NB. 0 < a < state size !
  double gamma = constants[3];
  double delta = constants[4];

  Variable u = control[0];  
  // NEUMANN: boundary conditions are y_x(0,t) = -c1*u and y_x(1,t)=0
  state_dynamics[0] = c0 * (- state[0] + state[1]) / h2 + c1 * u / h;
  state_dynamics[stateSize()-2] = c0 * (- state[stateSize()-2] + state[stateSize()-3]) / h2;
  for(int i=1;i<stateSize()-2;i++)
    state_dynamics[i] = c0 * (state[i-1] - 2*state[i] + state[i+1]) / h2;

  // CRITERION Min int (state^2 + gamma control + delta control^2)
  Variable normstate = 0.0;
  for(int i=0;i<stateSize()-2;i++)
    normstate = normstate + h*pow(state[i],2);

  state_dynamics[stateSize()-1] = normstate / 2.0 + gamma*u + delta*pow(u,2);

}

template <typename Variable>
inline void OCP::boundaryConditions(double initial_time, double final_time, const Variable *initial_state, const Variable *final_state, const Variable *parameters, const double *constants, Variable *boundary_conditions)
{
	// INITIAL CONDITIONS FOR HEAT PROBLEM
	for(int i=0;i<stateSize();i++)
    boundary_conditions[i] = initial_state[i];
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
