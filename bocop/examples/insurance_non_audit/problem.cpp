


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
  // maximize integral of utility over population
  final_cost = - final_state[3];
}

template <typename Variable>
inline void OCP::dynamics(double time, const Variable *state, const Variable *control, const Variable *parameters, const double *constants, Variable *state_dynamics)
{
  // constants
  double gamma = constants[0];
  double lambda = constants[1];
  double sigma = constants[6];
  int vtype = (int) constants[7];
  int ftype = (int) constants[8];
  double alpha = constants[9];

  // time, state and control
  double x = time;
  Variable I = state[0];
  Variable m = state[1];
  Variable h = control[0];

  // Expenses effect
  Variable vprime = 0e0;
  if (vtype == 0)
    //vprime = alpha * 1e0 / (2e0 * sqrt(alpha*m) * pow(1e0+sqrt(alpha*m),2e0)); // multiplicative alpha
    vprime = alpha/2 * pow(m,alpha/2-1) / pow(1+pow(m,alpha/2),2); // power alpha
  else if (vtype == 1)
    vprime = (log(m+2)/(m+1) - log(m+1)/(m+2)) / pow(log(m+2),2e0); // corner solution
  else
    std::cout << "Error (dynamics) >>> unknown vtype (should be 0 or 1): " << vtype << std::endl;

  // utility
  Variable U = control[3];
  Variable dUdR = control[4];

  // illness distribution
  Variable fx = 0e0;
  if (ftype == 0)
    fx = lambda * exp( - lambda * x) + exp( - lambda * final_time) / final_time;
  else if (ftype == 1)
    fx = 1e0 / (final_time - initial_time);
  else
    std::cout << "Error (dynamics) >>> unknown ftype (should be 0 or 1): " << ftype << std::endl;

  // dynamics
  state_dynamics[0] = (1e0 - gamma * x * vprime / dUdR) * h;
  state_dynamics[1] = h;
  state_dynamics[2] = (1e0 + sigma) * I * fx;
  state_dynamics[3] = U * fx;

}

template <typename Variable>
inline void OCP::boundaryConditions(double initial_time, double final_time, const Variable *initial_state, const Variable *final_state, const Variable *parameters, const double *constants, Variable *boundary_conditions)
{
  // I(0), P_int(0), J_int(0)
  boundary_conditions[0] = initial_state[0];
  boundary_conditions[1] = initial_state[2];
  boundary_conditions[2] = initial_state[3];

  // P_int(T) - P
  boundary_conditions[3] = final_state[2] - parameters[0];

  // m(0) CAREFUL WITH 'SQRT' FORMULATION FOR v(m) !
  boundary_conditions[4] = initial_state[1];
}

template <typename Variable>
inline void OCP::pathConstraints(double time, const Variable *state, const Variable *control, const Variable *parameters, const double *constants, Variable *path_constraints)
{
  
   // constants
  double gamma = constants[0];
  double h0 = constants[2];
  double w = constants[3];
  double s = constants[4];
  double k = constants[5];
  int vtype = (int) constants[7];
  double alpha = constants[9]; 

  // variables
  double x = time;
  Variable P = parameters[0];
  Variable I = state[0];
  Variable m = state[1];
  Variable R = control[1];
  Variable H = control[2];
  Variable U = control[3];
  Variable dUdR = control[4];

  // Revenue >= 0
  Variable epsilon = k * x / (final_time - x + 1);
  path_constraints[0] = R - (w - P + I - m - epsilon);

  // Expenses effect
  Variable v = 0e0;
  if (vtype == 0)
    //v = sqrt(alpha*m) / (1e0+sqrt(alpha*m)); // multiplicative alpha
    v = pow(m,alpha/2) / (1e0+pow(m,alpha/2)); // power alpha
  else if (vtype == 1)
    v = log(m+1) / log(m+2); // corner solution
  else
    std::cout << "Error (pathcond) >>> unknown vtype (should be 0 or 1): " << vtype << std::endl;

  // Health >= 0
  path_constraints[1] = H - (h0 - gamma * x * (1e0 - v));

  // Utility
  path_constraints[2] = U - (1e0 - exp( - s * R)+ H);

  // dUdR
  path_constraints[3] = dUdR - (s * exp( - s * R)); 

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
