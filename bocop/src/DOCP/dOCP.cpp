// dOCP.cpp
//

#include <dOCP.h>
#include <tools.h>


// dOCP implementation
dOCP::dOCP(void) : NLP(), rk(new dODE)
{
  // note: ocp is not initialized here since it is supposed to be created before dOCP and passed to it by setOCP
}

dOCP::~dOCP(void)
{
    delete rk;
}

void dOCP::initialize(void)
{

  if (!ocp)
  {
    std::cout << "dOCP::initialize() : Error: OCP is not set." << std::endl;
    exit(1);
  }

  // default solution name +++ here use prefix from .def file
  solution_file = "problem.sol";

  // PREPARE NLP TRANSCRIPTION
  // setup time discretization
  RKmethod = ocp->getDefinitionForKey("ode.discretization");  //+++ default value midpoint ?
  discretisation_steps = stoi(ocp->getDefinitionForKey("time.steps"));
  rk->setRKcoeffs(RKmethod);
  rk->setTimeGrids(ocp->OCP_initialTime(), ocp->OCP_finalTime(), discretisation_steps, time_step, time_step_grid, time_stage_grid);

  // setup NLP variables bounds and initial value
  variables_offset_state = 0;
  xd->setInitialState(time_step_grid, ocp, starting_point, variables_lower_bounds, variables_upper_bounds);
  variables_offset_control = starting_point.size();
  ud->setInitialControl(time_stage_grid, ocp, rk, starting_point, variables_lower_bounds, variables_upper_bounds);
  variables_offset_param = starting_point.size();
  NLP_parameters_size = rk->setInitialParam(ocp, starting_point, variables_lower_bounds, variables_upper_bounds);
  rk->setRKStageVars(discretisation_steps, ocp, starting_point, variables_lower_bounds, variables_upper_bounds);
  variables_size = starting_point.size();
  std::cout << "variables " << variables_size << " including parameters " << NLP_parameters_size << std::endl;


  // setup NLP constraints bounds
  rk->setBoundaryConditionsBounds(ocp, constraints_lower_bounds, constraints_upper_bounds);
  rk->setDiscretisedConstraintsBounds(discretisation_steps, ocp, constraints_lower_bounds, constraints_upper_bounds);
  constraints_size = constraints_lower_bounds.size();

}


void dOCP::writeSolution(const int status, const int iter, const double objective, const double constraints_viol, const double *variables, const double *multipliers, const double *constraints)
{

  // recover dimensions
  int dimState = ocp->stateSize();
  int dimControl = ocp->controlSize();
  int dimParameter = ocp->parametersSize();
  int dimBoundaryConditions = ocp->boundaryConditionsSize();
  int dimPathConstraints = ocp->pathConstraintsSize();
  int dimSteps = discretisationSteps();
  int dimStages = RKStages();

  // fill solution object
  dOCPsolution solution(dimState, dimSteps, dimControl, dimParameter, dimStages, dimBoundaryConditions, dimPathConstraints);
  solution.status = status;
  solution.iterations = iter;
  solution.constraint = constraints_viol;
  solution.objective = objective;
  xd->getState(variables, variables_offset_state, discretisationSteps(), ocp->stateSize(), *solution.state);
  ud->getControl(variables, variables_offset_control, discretisationSteps(), RKStages(), ocp->controlSize(), *solution.control);
  rk->getParam(variables, variables_offset_param, ocp->parametersSize(), *solution.parameter); 
  rk->getMultipliers(multipliers, *solution.boundary_conditions_multiplier, *solution.path_constraints_multiplier, *solution.adjoint_state);
  rk->getConstraints(constraints, *solution.boundary_conditions, *solution.path_constraints, *solution.dyn_equations);

  // (backup and) open solution file
  bool exists;
  std::ifstream file_check(solution_file.c_str());
  exists = !file_check.fail();
  file_check.close();
  if (exists)
  {
    std::string newName = solution_file + ".backup";
    rename(solution_file.c_str(),newName.c_str());
  }
  std::ofstream file_out(solution_file.c_str(), std::ios::out | std::ios::binary);
  if (!file_out)
  {
    std::cerr << std::endl << "ERROR >>> dOCP::writeSolution() : cannot open solution file " << solution_file << std::endl << std::endl;
    exit(1);
  }
  file_out.precision(15);

  // copy of .def (NB. dictionary will be saved in alphabetical order :p)
  ocp->save(file_out);

  // status, iter, objective and constraints violation
  file_out << "########################################################################" << std::endl;
  file_out << "# SOLUTION" << std::endl;
  file_out << "########################################################################" << std::endl;

  file_out << SOLVER_STATUS_TITLE << solution.status << std::endl;
  file_out << SOLVER_ITERATIONS_TITLE << solution.iterations << std::endl;
  file_out << OBJECTIVE_FUNCTION_TITLE << solution.objective << std::endl;
  file_out << CONSTRAINT_VIOLATION_TITLE << solution.constraint << std::endl;
  file_out << std::endl;

  // times (denormalize time grids if needed) 
  // +++ cant seem to be able to reuse the dOCP getter for final time, type mismatch for v -_-, see also below
  if (ocp->hasFreeFinalTime())
  {
    double t0 = initialTime();
    double tf = variables[variables_offset_param];
    
    std::vector<double> true_time_step_grid(time_step_grid.size());
    for (std::size_t i = 0; i<time_step_grid.size(); i++)
      true_time_step_grid[i] = t0 + time_step_grid[i] * (tf - t0);
    bcp::writeDataBlock1D(file_out, TIME_STEPS_TITLE, true_time_step_grid);
  
    std::vector<double> true_time_stage_grid(time_stage_grid.size());  
    for (std::size_t i = 0; i<time_stage_grid.size(); i++)
      true_time_stage_grid[i] = t0 + time_stage_grid[i] * (tf - t0);
    bcp::writeDataBlock1D(file_out, TIME_STAGES_TITLE, true_time_stage_grid);      
  }
  else
  {
    bcp::writeDataBlock1D(file_out, TIME_STEPS_TITLE, time_step_grid);
    bcp::writeDataBlock1D(file_out, TIME_STAGES_TITLE, time_stage_grid);
  }

  // variables
  bcp::writeDataBlock2D(file_out, STATE_TITLE, *solution.state);
  bcp::writeDataBlock2D(file_out, CONTROL_TITLE, *solution.control);
  bcp::writeDataBlock1D(file_out, PARAMETER_TITLE, *solution.parameter);

  // constraints
  bcp::writeDataBlock1D(file_out, BOUNDARY_TITLE, *solution.boundary_conditions);
  bcp::writeDataBlock2D(file_out, PATH_CONSTRAINTS_TITLE, *solution.path_constraints);

  // multipliers
  file_out << std::endl;
  bcp::writeDataBlock1D(file_out, MULTI_BOUNDARY_CONDITION_TITLE, *solution.boundary_conditions_multiplier);
  bcp::writeDataBlock2D(file_out, MULTI_PATH_CONSTRAINTS_TITLE, *solution.path_constraints_multiplier);
  bcp::writeDataBlock2D(file_out, ADJOINT_STATE_TITLE, *solution.adjoint_state);

  // average control (on steps) (NB. cannot reuse controlAtStep because fscking template)
  std::vector<std::vector <double>> avg_control(dimControl,std::vector<double>(dimSteps));
  for (int k=0;k<dimSteps;k++)
    for (std::size_t j = 0; j < RKStages(); ++j)
        for (int i=0;i<dimControl;i++)
            avg_control[i][k] += rk->butcherB()[j] * (*solution.control)[i][k*RKStages()+j];

  bcp::writeDataBlock2D(file_out, "# Average control on steps", avg_control);  
  

  //+++ add kstage



}



//
// dOCP.cpp ends here
