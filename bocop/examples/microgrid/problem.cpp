


// +++DRAFT+++ This class implements the OCP functions
// It derives from the generic class bocop3OCPBase
// OCP functions are defined with templates since they will be called
// from both the NLP solver (double arguments) and AD tool (ad_double arguments)
//#pragma once

#include <OCP.h>

// ///////////////////////////////////////////////////////////////////

// interpolation
int DataSize = 193;
std::vector<double> TimeGrid;
std::vector<double> SolarPower;
std::vector<double> LoadPower;


template <typename Variable>
inline void OCP::finalCost(double initial_time, double final_time, const Variable *initial_state, const Variable *final_state, const Variable *parameters, const double *constants, Variable &final_cost)
{
  
  // select optional terms
  int enable_reg = (int) constants[7];
  int battery_aging = (int) constants[8];

  // minimize total operating cost, base i diesel consumption
  final_cost = final_state[1];

  // battery aging
  if (battery_aging == 2)
     final_cost += final_state[2];

  // quadratic regularization
  if (enable_reg == 1) 
     final_cost += final_state[3];

}

template <typename Variable>
inline void OCP::dynamics(double time, const Variable *state, const Variable *control, const Variable *parameters, const double *constants, Variable *state_dynamics)
{
  
  //constants for SOC dynamics
  double rho = constants[0];
  double capacity_bat = constants[1];
  double U_bat = constants[2];
  double Ah_bat = constants[3];
  double Cost_bat = constants[4];
  double reg_coeff = constants[5];

  // controls
  Variable P_diesel = control[0];
  Variable P_in = control[1];
  Variable P_out = control[2];

  // diesel consumption model: K_D * (P_D + eps)^0.9, fitted to measured data
  // epsilon added to avoid non-differentiability at P_D=0 (warning 'evluation error')
  double epsilon = 1e-4;
  double K_D = 0.471426e0;
  double C_diesel = 500e0; //price of 1 l in CPL$

  // cost for default power
  Variable slack_default = control[4];
  double C_default = 250e0; //price of 1kWh in CPL$

  // retrieve output current for battery ageing (Loss of Life in Ah)
  Variable I_out = P_out * 1e3 / U_bat;
  Variable SOC = state[0];
  Variable SF = (-4e0*SOC*SOC + 5e0) / 5e0; //severity factor 

  // dynamics for SOC
  state_dynamics[0] = (P_in*rho - P_out) / capacity_bat; 

  // running cost: diesel fuel + default power + regularization + battery LoL
  state_dynamics[1] = C_diesel * K_D * pow(P_diesel + epsilon,0.9e0) + C_default * slack_default;

  // running cost: battery Loss of life (model 2) 
  state_dynamics[2] = Cost_bat / Ah_bat * SF * I_out;

  // quadratic regularization term 
  state_dynamics[3] = reg_coeff * P_out * P_out;

}

template <typename Variable>
inline void OCP::boundaryConditions(double initial_time, double final_time, const Variable *initial_state, const Variable *final_state, const Variable *parameters, const double *constants, Variable *boundary_conditions)
{

  // SOC periodicity
  boundary_conditions[0] = final_state[0] - initial_state[0];

  // Initial cost set to 0
  boundary_conditions[1] = initial_state[1];
  boundary_conditions[3] = initial_state[2];
  boundary_conditions[4] = initial_state[3];

  // SOC(0) - SOC_0
  boundary_conditions[2] = initial_state[0] - constants[5];

  // retrieve cost terms
  boundary_conditions[5] = parameters[0] - final_state[1];
  boundary_conditions[6] = parameters[1] - final_state[2];
  boundary_conditions[7] = parameters[2] - final_state[3];

}

template <typename Variable>
inline void OCP::pathConstraints(double time, const Variable *state, const Variable *control, const Variable *parameters, const double *constants, Variable *path_constraints)
{
  // use interpolated data
  int verbose = 1;
  double P_solar = bcp::interpolation1Dlinear(time, TimeGrid, SolarPower, DataSize, verbose);
  double P_load = bcp::interpolation1Dlinear(time, TimeGrid, LoadPower, DataSize, verbose);

  // controls
  Variable P_diesel = control[0];
  Variable P_in = control[1];
  Variable P_out = control[2];
  Variable slack_excess = control[3];
  Variable slack_default = control[4];

  // power equilibrium
  path_constraints[0] = P_diesel + P_solar + P_out - P_load - P_in - slack_excess + slack_default;

  // Visualization
  path_constraints[1] = P_solar;
  path_constraints[2] = P_load;
}

void OCP::preProcessing()
{
  // set time grid for interpolated data
  printf("%d\n", DataSize);
  TimeGrid.resize(DataSize);
  for (int i=0; i<TimeGrid.size(); i++)
    TimeGrid[i] = (double) i / 4e0;

	//read solar power and power load data
  // NB. constants are not available here since preprocessing is called at OCP initialization, before reading the .def file -_-
  printf("Read solar and load power data...\n");
  int verbose = 1;
//  bcp::readFileToVector("data/Summer-Solar.data", SolarPower, verbose);
//  bcp::readFileToVector("data/Summer-Load.data", LoadPower, verbose);    
  bcp::readFileToVector("data/Winter-Solar.data", SolarPower, verbose);
  bcp::readFileToVector("data/Winter-Load.data", LoadPower, verbose);


}

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
