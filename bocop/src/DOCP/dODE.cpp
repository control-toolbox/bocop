// dODE.cpp
//

#include <iostream>

#include <dODE.h>


void dODE::initialize()
{
}


void dODE::setRKcoeffs(const std::string RKmethod)
{
    if (!RKmethod.compare("euler_implicit"))
    {
        // Euler implicit (1-stage, order 1)
        //  1    1
        //       1
        rk_stages = 1;
        butcher_a.resize(rk_stages, std::vector<double>(rk_stages));
        butcher_b.resize(rk_stages);
        butcher_c.resize(rk_stages);
        butcher_a[0][0] = 1e0;
        butcher_b[0] = 1e0;
        butcher_c[0] = 1e0;
    }
    else if (!RKmethod.compare("midpoint_implicit"))
    {
        // Midpoint implicit (1-stage, order 2)
        // 1/2  1/2
        //        1
        rk_stages = 1;
        butcher_a.resize(rk_stages, std::vector<double>(rk_stages));
        butcher_b.resize(rk_stages);
        butcher_c.resize(rk_stages);
        butcher_a[0][0] = 0.5e0;
        butcher_b[0] = 1e0;
        butcher_c[0] = 0.5e0;
    }
    else if (!RKmethod.compare("trapeze"))
    {
        // Explicit trapeze (2-stage, order 2)
        //  0     0       0
        //  1   1/2     1/2
        //      1/2     1/2
        rk_stages = 2;
        butcher_a.resize(rk_stages, std::vector<double>(rk_stages));
        butcher_b.resize(rk_stages);
        butcher_c.resize(rk_stages);
        butcher_a[0][0] = 0e0;
        butcher_a[0][1] = 0e0;
        butcher_a[1][0] = 0.5e0;
        butcher_a[1][1] = 0.5e0;
        butcher_b[0] = 0.5e0;
        butcher_b[1] = 0.5e0;
        butcher_c[0] = 0e0;
        butcher_c[1] = 1e0;        
    }
    else if (!RKmethod.compare("gauss2"))
    {
        // Gauss s=2 aka Hammer and Hollingworth (2-stage, order 4)
        // A stable, B stable, symplectic. Geometric Numerical Integration Table 1.1 p34
        // 1/2 - sqrt(3)/6   1/4              1/4 - sqrt(3)/6
        // 1/2 + sqrt(3)/6   1/4 + sqrt(3)/6  1/4
        //                      1/2              1/2
        rk_stages = 2;
        butcher_a.resize(rk_stages, std::vector<double>(rk_stages));
        butcher_b.resize(rk_stages);
        butcher_c.resize(rk_stages);
        butcher_a[0][0] = 0.25e0;
        butcher_a[0][1] = 0.25e0 - sqrt(3e0)/6e0;
        butcher_a[1][0] = 0.25e0 + sqrt(3e0)/6e0;
        butcher_a[1][1] = 0.25e0;
        butcher_b[0] = 0.5e0;
        butcher_b[1] = 0.5e0;
        butcher_c[0] = 0.5e0 - sqrt(3e0)/6e0;
        butcher_c[1] = 0.5e0 + sqrt(3e0)/6e0;
    }

    else if (!RKmethod.compare("gauss3"))
    {    
        // Gauss s=3 aka Kuntzmann and Butcher (3-stage, order 6)
        // A stable, B stable, symplectic. Geometric Numerical Integration Table 1.1 p34
        //  1/2 - sqrt(15)/10       5/36                2/9-sqrt(15)/15     5/36-sqrt(15)/30
        //  1/2                       5/36+sqrt(15)/24  2/9                   5/36-sqrt(15)/24
        //  1/2 + sqrt(15)/10       5/36+sqrt(15)/30  2/9+sqrt(15)/15    5/36
        //                              5/18                4/9                5/18
        rk_stages = 3;
        butcher_a.resize(rk_stages, std::vector<double>(rk_stages));
        butcher_b.resize(rk_stages);
        butcher_c.resize(rk_stages);
        butcher_a[0][0] = 5e0/36e0;
        butcher_a[0][1] = 2e0/9e0 - sqrt(15e0) / 15e0;
        butcher_a[0][2] = 5e0/36e0 - sqrt(15e0) / 30e0;
        butcher_a[1][0] = 5e0/36e0 + sqrt(15e0) / 24e0;
        butcher_a[1][1] = 2e0/9e0;
        butcher_a[1][2] = 5e0/36e0 - sqrt(15e0) / 24e0;
        butcher_a[2][0] = 5e0/36e0 + sqrt(15e0) / 30e0;
        butcher_a[2][1] = 2e0/9e0 + sqrt(15e0) / 15e0;
        butcher_a[2][2] = 5e0/36e0;        
        butcher_b[0] = 5e0 / 18e0;
        butcher_b[1] = 4e0 / 9e0;
        butcher_b[2] = 5e0 / 18e0;        
        butcher_c[0] = 0.5e0 - sqrt(15e0) / 10e0;
        butcher_c[1] = 0.5e0;
        butcher_c[2] = 0.5e0 + sqrt(15e0) / 10e0;
    }
    else
    {
        std::cout << "ERROR: unknown RK method: " << RKmethod << std::endl;
        exit(1);
    }
}


void dODE::setTimeGrids(const double t0, const double tf, const std::size_t discretisation_steps, double &time_step, std::vector<double> &time_step_grid, std::vector<double> &time_stage_grid)
{

  // fill time step/stage vectors
  time_step_grid.resize(discretisation_steps + 1);
  time_stage_grid.resize(discretisation_steps * rk_stages);
  time_step = (tf - t0) / discretisation_steps;
  for (size_t i=0; i < discretisation_steps; ++i)
  {
    time_step_grid[i] = t0 + i*time_step;
    for (size_t j=0; j < rk_stages; ++j)
      time_stage_grid[i * rk_stages+j] = time_step_grid[i] + butcher_c[j]*time_step;
  }
  time_step_grid[discretisation_steps] = tf;

}


void dODE::setInitialParam(OCP *ocp, std::vector<double> &starting_point, std::vector<double> &variables_lower_bounds, std::vector<double> &variables_upper_bounds)
{
  // set OCP parameters
  for (size_t k = 0; k < ocp->parametersSize(); ++k)
  {
    // set bounds
    variables_lower_bounds.push_back(ocp->paramLowerBounds()[k]);
    variables_upper_bounds.push_back(ocp->paramUpperBounds()[k]);

    // set initial value from .def file
    std::stringstream label;
    label << "parameter." << k << ".init";
    std::string param_init_type = ocp->getDefinitionForKey(label.str(), "0.1");
    starting_point.push_back(stod(param_init_type));
  }
 
/*  
  // additional parameter for free final time
  if (ocp->hasFreeFinalTime())
  {
    std::cout << "set additional parameter for free final time" << std::endl;
    total_param_size ++;
    // set lower bound to max(t0, 1e-3)
    variables_lower_bounds.push_back(std::max(ocp->OCP_initialTime(), 1e-3));
    variables_upper_bounds.push_back(2e20);
    // set initial value to 1 
    starting_point.push_back(1.0);
  }
 */
}


void dODE::setRKStageVars(size_t discretisation_steps, OCP *ocp, std::vector<double> &starting_point, std::vector<double> &variables_lower_bounds, std::vector<double> &variables_upper_bounds)
{
  // set RK stage variables initialisation
  for (size_t i = 0; i < discretisation_steps; ++i)
    for (size_t j = 0; j < RKStages(); ++j)
      for (size_t k = 0; k < ocp->stateSize(); k++)
      {
        variables_lower_bounds.push_back(std::numeric_limits<double>::lowest());
        variables_upper_bounds.push_back(std::numeric_limits<double>::max());
        starting_point.push_back(0e0);
      }
}


// Layout of NLP constraints C for discretized OCP 
// {boundarycond [dynstep (dynstage...dynstage) pathcond] ... [dynstep (dynstage...dynstage) pathcond] pathcond_tf}
void dODE::setBoundaryConditionsBounds(OCP *ocp, std::vector<double> &constraints_lower_bounds, std::vector<double> &constraints_upper_bounds)
{
  for (size_t i = 0; i < ocp->boundaryConditionsSize(); ++i)
  {
    constraints_lower_bounds.push_back(ocp->boundaryLowerBounds()[i]);
    constraints_upper_bounds.push_back(ocp->boundaryUpperBounds()[i]);
  }
}

void dODE::setDiscretisedConstraintsBounds(size_t discretisation_steps, OCP *ocp, std::vector<double> &constraints_lower_bounds, std::vector<double> &constraints_upper_bounds)
{
  
  // main loop over time steps
  for (size_t l = 0; l < discretisation_steps; ++l)
  {
    // 1. dynamics constraint at time step: y_l + h sum(b_j*k_j) - y_{l+1} = 0
    for (size_t i = 0; i < ocp->stateSize(); ++i)
    {
      constraints_lower_bounds.push_back(0e0);
      constraints_upper_bounds.push_back(0e0);
    }

    // 2. loop on stages for k_j equations: f(...) - k_j = 0
    for (size_t j = 0; j < RKStages(); ++j)
      for (size_t i = 0; i < ocp->stateSize(); ++i)
      {
        constraints_lower_bounds.push_back(0e0);
        constraints_upper_bounds.push_back(0e0);
      }

    // 3. path constraints (on step with average control)
    for (size_t i = 0; i < ocp->pathConstraintsSize(); ++i)
    {
      constraints_lower_bounds.push_back(ocp->pathLowerBounds()[i]);
      constraints_upper_bounds.push_back(ocp->pathUpperBounds()[i]);
    }
  } // end main loop over time steps
  
  // add path constraints at final time
  for (size_t i = 0; i < ocp->pathConstraintsSize(); ++i)
  {
    constraints_lower_bounds.push_back(ocp->pathLowerBounds()[i]);
    constraints_upper_bounds.push_back(ocp->pathUpperBounds()[i]);
  }

}

void dODE::getParam(const double* x, size_t param_offset, size_t dimParam, std::vector<double>& parameter)
{
  size_t index_x = param_offset;
  for (size_t j = 0; j < dimParam; j++)
    parameter[j] = x[index_x++];
}

// extract multipliers for the constraints C(X) of (NLP)
// Layout of C: {boundarycond [dynstep (dynstage...dynstage) pathcond] ... [dynstep (dynstage...dynstage) pathcond] pathcond_tf}
void dODE::getMultipliers(const double* lambda, std::vector<double>& boundaryCondMultiplier, std::vector<std::vector<double> >&  pathConstrMultiplier, std::vector<std::vector<double> >&  adjointState)
{
  // retrieve dimensions
  int dimState = adjointState.size();
  int dimPathConstraints = pathConstrMultiplier.size();
  int dimSteps = adjointState[0].size();
  int dimStages = RKStages();

  int index_lambda = 0;

  // boundary conditions
  for (int j = 0; j < boundaryCondMultiplier.size(); j++)
    boundaryCondMultiplier[j] = lambda[index_lambda++];

  // loop on time steps: block [dynstep (dynstage stage 1) ... (dynstage stage s) pathcond]
  for (int i = 0; i < dimSteps; i++)
  {
    // multipliers for dynamics equation a.k.a 'adjoint State'
    for (int j = 0; j < dimState; j++)
      adjointState[j][i] = lambda[index_lambda++];

    // skip multipliers for stage equations
    for (int j = 0; j < dimStages; j++)
      index_lambda += dimState;

    // multipliers for path constraints
    for (int k = 0; k < dimPathConstraints; k++)
      pathConstrMultiplier[k][i] = lambda[index_lambda++];
  }
  
  // pathcond at final time
  for (int k = 0; k < dimPathConstraints; k++)
    pathConstrMultiplier[k][dimSteps] = lambda[index_lambda++];
}


// extract components from constraints C(X) of (NLP)
// Layout of C: {boundarycond [dynstep (dynstage...dynstage) pathcond] ... [dynstep (dynstage...dynstage) pathcond] pathcond_tf}
// where boundarycond is the constraint: lb <= \phi(y^0,Y^N) <= ub
// dynstep is the dynamics equation: y^{i+1} - ( y^i + sum_{i=1..s} b_j k_j ) = 0
// dynstage are the s equations at stages: k^i_j - f(t_i + c_i h, y^i + sum_{i=1..s} a_lj k_l, u^i_j) = 0     //check indexes for a +++
// pathcond are the constraints at steps: LB <= g(t_i, y^i, u^i) <= UB   with  u^i e.g. the 'average' control on step
void dODE::getConstraints(const double* g, std::vector<double>& boundaryConditions, std::vector<std::vector<double> >&  pathConstraints, std::vector<std::vector<double> >& dynEquations)
{
  // retrieve dimensions
  int dimState = dynEquations.size();
  int dimPathConstraints = pathConstraints.size();
  int dimSteps = dynEquations[0].size();
  int dimStages = RKStages();

  int index_g = 0;

  // boundary conditions
  for (int j = 0; j < boundaryConditions.size(); j++)
    boundaryConditions[j] = g[index_g++];

  // loop on time steps: block [dynstep (dynstage stage 1) ... (dynstage stage s) pathcond]
  for (int i = 0; i < dimSteps; i++)
  {
    // dynamics equation
    for (int j = 0; j < dimState; j++)
      dynEquations[j][i] = g[index_g++];

    // skip stage equations
    for (int j = 0; j < dimStages; j++)
      index_g += dimState;

    // path constraints
    for (int k = 0; k < dimPathConstraints; k++)
      pathConstraints[k][i] = g[index_g++];

  }
  
  // path constraints at final time
  for (int k = 0; k < dimPathConstraints; k++)
    pathConstraints[k][dimSteps] = g[index_g++];  

}


//
// dODE.cpp ends here
