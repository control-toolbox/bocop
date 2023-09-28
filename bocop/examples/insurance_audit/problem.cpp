


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
  final_cost = - final_state[7];
}

template <typename Variable>
inline void OCP::dynamics(double time, const Variable *state, const Variable *control, const Variable *parameters, const double *constants, Variable *state_dynamics)
{

  // constants and parameters
  double gamma = constants[0];
  double lambda = constants[1];
  double a = constants[5]; // needed since we rescaled time on [0,1]
  double c = constants[6];
  double sigma = constants[7];
  int vtype = (int) constants[8];
  Variable xstar = parameters[1];

  // PHASE 1
  // time, state and control
  Variable x1 = time * xstar;
  Variable I1 = state[0];
  Variable m1 = state[1];
  Variable h1 = control[0];

  // Expenses effect
  Variable vprime1 = 0e0;
  if (vtype == 0)
    vprime1 = 1e0 / (2e0 * sqrt(m1) * pow(1e0+sqrt(m1),2e0)); //suppresses corner solution
  else if (vtype == 1)
    vprime1 = (log(m1+2)/(m1+1) - log(m1+1)/(m1+2)) / pow(log(m1+2),2e0);
  else
    std::cout << "Error (dynamics) >>> unknown vtype (should be 0 or 1): " << vtype << std::endl;

  // utility
  Variable U1 = control[5];
  Variable dUdR1 = control[6];

  // illness distribution
  Variable fx1 = lambda * exp( - lambda * x1);

  // dynamics
  state_dynamics[0] = (1e0 - gamma * x1 * vprime1 / dUdR1) * h1;
  state_dynamics[1] = h1;
  state_dynamics[2] = (1e0 + sigma) * I1 * fx1;
  state_dynamics[3] = U1 * fx1;

  // time normalization
  for (int i=0;i<4;i++)
     state_dynamics[i] *= xstar; 


  // PHASE 2
  // time, state and control
  Variable x2 = xstar + time * (a - xstar);
  Variable I2 = state[4];
  Variable h2 = control[1];
  Variable g2 = control[2]; // in [0,1]

  // utility
  Variable U2 = control[9];

  // illness distribution
  Variable epsilonfx = exp( - lambda * final_time) / final_time;
  Variable fx2 = lambda * exp( - lambda * x2);

  // dynamics
  state_dynamics[4] = g2 * h2;           // free value below h
  state_dynamics[5] = h2;
  state_dynamics[6] = (1e0 + sigma) * (I2 + c) * fx2;  // audit cost
  state_dynamics[7] = U2 * fx2;

  // time normalization
  for (int i=4;i<8;i++)
     state_dynamics[i] *= (a - xstar); 

}

template <typename Variable>
inline void OCP::boundaryConditions(double initial_time, double final_time, const Variable *initial_state, const Variable *final_state, const Variable *parameters, const double *constants, Variable *boundary_conditions)
{
  
  // I(0), m(0), P_int(0), J_int(0)
  boundary_conditions[0] = initial_state[0];
  boundary_conditions[1] = initial_state[1];
  boundary_conditions[2] = initial_state[2];
  boundary_conditions[3] = initial_state[3];

  // P_int(T) - P
  boundary_conditions[4] = final_state[6] - parameters[0];

  // junction phase 1-2 for I, m, P, J
  boundary_conditions[5] = initial_state[4] - final_state[0];
  boundary_conditions[6] = initial_state[5] - final_state[1];
  boundary_conditions[7] = initial_state[6] - final_state[2];
  boundary_conditions[8] = initial_state[7] - final_state[3];

  // upper bound on jump ie I(x*+) - I(x*-) <= m(x*+) - m(x*-)
  boundary_conditions[9] = (initial_state[4] - final_state[0]) - (initial_state[5] - final_state[1]);

}

template <typename Variable>
inline void OCP::pathConstraints(double time, const Variable *state, const Variable *control, const Variable *parameters, const double *constants, Variable *path_constraints)
{

  // constants and parameters
  double gamma = constants[0];
  double h0 = constants[2];
  double w = constants[3];
  double s = constants[4];
  double a = constants[5];
  int vtype = (int) constants[8];
  Variable P = parameters[0];
  Variable xstar = parameters[1];

  // PHASE 1
  // variables
  Variable x1 = time * xstar;
  Variable I1 = state[0];
  Variable m1 = state[1];
  Variable R1 = control[3];
  Variable H1 = control[4];
  Variable U1 = control[5];
  Variable dUdR1 = control[6];

  // Revenue >= 0 (no background risk)
  path_constraints[0] = R1 - (w - P + I1 - m1);

  // Expenses effect
  Variable v1 = 0e0;
  if (vtype == 0)
    v1 = sqrt(m1) / (1e0+sqrt(m1));  //suppresses corner solution
  else if (vtype == 1)
    v1 = log(m1+1) / log(m1+2);
  else
    std::cout << "Error (pathcond) >>> unknown vtype (should be 0 or 1): " << vtype << std::endl;

  // Health >= 0
  path_constraints[1] = H1 - (h0 - gamma * x1 * (1e0 - v1));

  // Utility
  path_constraints[2] = U1 - (1e0 - exp( - s * R1)+ H1);

  // dUdR
  path_constraints[3] = dUdR1 - (s * exp( - s * R1));


  // PHASE 2
  // variables
  Variable x2 = xstar + time * (a - xstar);
  Variable I2 = state[4];
  Variable m2 = state[5];
  Variable R2 = control[7];
  Variable H2 = control[8];
  Variable U2 = control[9];
  Variable dUdR2 = control[10];

  // Revenue >= 0 (no background risk)
  path_constraints[4] = R2 - (w - P + I2 - m2);

  // Expenses effect
  Variable v2 = 0e0;
  if (vtype == 0)
    v2 = sqrt(m2) / (1e0+sqrt(m2));  //suppresses corner solution
  else if (vtype == 1)
    v2 = log(m2+1) / log(m2+2);
  else
    std::cout << "Error (pathcond) >>> unknown vtype (should be 0 or 1): " << vtype << std::endl;

  // Health >= 0
  path_constraints[5] = H2 - (h0 - gamma * x2 * (1e0 - v2));

  // Utility
  path_constraints[6] = U2 - (1e0 - exp( - s * R2)+ H2);

  // dUdR
  path_constraints[7] = dUdR2 - (s * exp( - s * R2));

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
