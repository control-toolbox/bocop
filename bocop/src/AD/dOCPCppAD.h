// dOCPCppAD.h
//

#pragma once

#include <cppad/cppad.hpp>
using double_ad = CppAD::AD<double>;
using fun_ad = CppAD::ADFun<double>;
template <typename T> using vector_ad = CppAD::vector<T>;
using sparse_jac_work = CppAD::sparse_jacobian_work;
using sparse_hess_work = CppAD::sparse_hessian_work;


#include <dOCP.h>
/** ***************************************************************************
* \class dOCPCppAD
* \brief dOCPCppAD implements the direct transcription that rewrites OCP as an NLP, providing the required functions and derivatives
*
* It derives from the generic class dOCP and interfaces with the automatic differentiation tool CppAD
* - uses a generalized Runge Kutta formula (defined with Butcher tables)
* - defines NLP functions: Objective F, Constraints G and derivatives gradF, jacG (sparse), hessL (sparse)
*
* Note: here and at the OCP level several functions are defined with templates, for both double and CppAD::AD<double>
* ****************************************************************************/
class dOCPCppAD : public dOCP
{
public:
  dOCPCppAD(void);
  ~dOCPCppAD(void);

  void initialize(void) override; // overrides the one from NLP or dOCP ?

  /** \name  NLP functions and derivatives (overrides for NLP class) */
  /**@{*/
  bool evalObjective(const double *variables, double& objective) override;
  bool evalConstraints(const double *variables, double *constraints) override;
  bool evalLagrangian(const double *primal_dual_variables, double& lagrangian) override;

  bool evalObjectiveGradient(const double *variables, double *grad_objective) override;
  bool evalConstraintsJacobian(const double *variables, double *jac_constraints) override;
  bool evalLagrangianHessian(const double *variables, double obj_factor, const double *lambda, double *lag_hessian) override;
  /**@}*/

  std::size_t jacobianNonZeroEntries(void) const override { return row_jac.size(); }
  const std::size_t *jacobianRowIndices(void) const override { return row_jac.data(); }
  const std::size_t *jacobianColIndices(void) const override { return col_jac.data(); }
  bool setConstraintsJacobianSparsityPattern(int *row_indices, int *col_indices, int jacobian_nonzero) override;

  std::size_t hessianNonZeroEntries(void) const override { return row_hess.size(); }
  const std::size_t *hessianRowIndices(void) const override { return row_hess.data(); }
  const std::size_t *hessianColIndices(void) const override { return col_hess.data(); }
  bool setLagrangianHessianSparsityPattern(int *row_indices, int *col_indices, int hessian_nonzero) override;

  // auxiliary functions for derivatives
  template <typename Variable> bool evalObjective_t(const Variable& v, Variable& o);
  template <typename Variable> bool evalConstraints_t(const Variable& v, Variable& g);
  template <typename Variable> bool evalLagrangian_t(const Variable& primal_dual_v, Variable& lag);
  void initializeObjectiveGradient(const double *x);
  void initializeConstraintsJacobian(const double *x);
  void initializeLagrangianHessian(const double *x);


public:

  // retape = 0: reuse tapes for each evaluation (standard case: no branching wrt Variable values)
  // retape = 1; reuse sparsity pattern but recompute tape for each evaluation
  //             required for instance to have proper derivatives when branching eg interpolation
  // (retape = 2; recompute tape and sparsity pattern at each evaluation ? NB probably need to signal ipopt)
  int ad_retape;

  std::vector<double_ad> x_ad;

  // gradient objective
  std::vector<double_ad> objvalue_ad;
  fun_ad f_obj;
  std::vector<double> grad;

  // constraints jacobian
  std::vector<double_ad> g_ad;
  fun_ad g_con;
  vector_ad<std::set<size_t>> pattern_jac;
  vector_ad<size_t> col_jac, row_jac;
  sparse_jac_work work_jac;
  std::vector<double> x2;
  std::vector<double> jac;

  // lagrangian hessian
  std::vector<double_ad> lagvalue_ad;
  fun_ad h_lag;
  vector_ad<std::set<std::size_t> > pattern_hess;
  vector_ad<std::size_t> col_hess, row_hess;
  sparse_hess_work work_hess;
  std::vector<double> xlf2, hess;

};

template <typename Variable>
inline bool dOCPCppAD::evalObjective_t(const Variable& v, Variable& o)
{

  double initial_time = initialTime();
  double final_time = ocp->OCP_finalTime();
  //auto final_time = finalTime(v);
  auto initial_state = stateAtStep(v, 0);
  auto final_state = stateAtStep(v, discretisationSteps());
  auto parameters = getParameters(v);
  auto constants = ocp->getConstants();
  ocp->finalCost(initial_time, final_time, initial_state.data(), final_state.data(), parameters.data(), constants.data(), o[0]);

  return true;
}

// Layout of NLP constraints C for discretized OCP
// {boundarycond [dynstep (dynstage...dynstage) pathcond] ... [dynstep (dynstage...dynstage) pathcond] pathcond_tf}
template <typename Variable>
inline bool dOCPCppAD::evalConstraints_t(const Variable& v, Variable& g)
{
  // NB for more genericity use aux functions fillDynamicsConstraints, fillPathConstraints etc in dODE ?
  using value_t = typename Variable::value_type;

  // general index for constraints vector
  std::size_t index = 0;
  
  // time discretization  
  double initial_time = initialTime();
  double final_time = ocp->OCP_finalTime();
  double h = timeStep();

  // 1. boundary conditions
  auto final_state = stateAtStep(v, discretisationSteps());
  auto parameters = getParameters(v);
  auto constants = ocp->getConstants();
  ocp->boundaryConditions(initial_time, final_time, stateAtStep(v, 0).data(), final_state.data(), parameters.data(), constants.data(), &g[index]);
  index += ocp->boundaryConditionsSize();

  // 2. loop over steps: discretized dynamics + path constraints
  std::vector<value_t> state_dynamics(ocp->stateSize());
  for (std::size_t l = 0; l < discretisationSteps(); ++l)
  {
    // 2.1 dynamics constraint at time step: y_l + h sum(b_j*k_j) - y_{l+1} = 0
    auto step_state = stateAtStep(v, l);
    auto next_step_state = stateAtStep(v, l+1);
    for (std::size_t i = 0; i < ocp->stateSize(); ++i)
    {
      value_t sum_bk_i = 0e0;
      for (std::size_t j = 0; j < RKStages(); ++j)
        sum_bk_i += rk->butcherB()[j] * kComponent(v, l, j, i);

      // NB. here we have 2 possible choices for the 'sign' of the equality constraint
      // this formulation is consistent with multipliers for initial conditions x=x0
      if (ocp->hasFreeFinalTime())
        // factor for rescaling time interval from [t0,1] to [t0,tf] +++ check case when t0 is different from 0 !
        g[index++] = next_step_state[i] - (step_state[i] + h * (finalTime(v) - initial_time) / (1.0 - initial_time) * sum_bk_i);
        //g[index++] = next_step_state[i] - (step_state[i] + h * v[variables_offset_param] * sum_bk_i);
      else 
        g[index++] = next_step_state[i] - (step_state[i] + h * sum_bk_i);      
    }

    // 2.2 loop on stages for k_j equations: f(...) - k_j = 0
    for (std::size_t j = 0; j < RKStages(); ++j)
    {
      //std::cout << "time stage for dynamics " << timeAtStage(v, l, j) << std::endl;
      ocp->dynamics(timeAtStage(l, j), stateAtStage(v, l, j).data(), controlAtStage(v, l, j).data(), parameters.data(), constants.data(), state_dynamics.data());
      for (std::size_t i = 0; i < ocp->stateSize(); ++i)
        g[index++] = state_dynamics[i] - kComponent(v, l, j, i);          
    }

    // 2.3 path constraints (on step with average control)
    ocp->pathConstraints(timeAtStep(l), step_state.data(), controlAtStep(v, l).data(), parameters.data(), constants.data(), &g[index]);
    index += ocp->pathConstraintsSize();

  } // end main loop over time steps
  
  // 3 add path constraints at final time (since main loop stops at penultimate time step); reuse average control for last time step
  ocp->pathConstraints(final_time, final_state.data(), controlAtStep(v, discretisationSteps()-1).data(), parameters.data(), constants.data(), &g[index]);
  index += ocp->pathConstraintsSize();
  
  // check final index filled vs constraints vector size  
  if (index != constraintsSize())
  {
    std::cout << "dOCPCppAD::evalConstraints_t >>> constraints last index " << index << " vs total size " << constraintsSize() << std::endl;
    exit(1);
  }

  return true;

}



template <typename Variable>
inline bool dOCPCppAD::evalLagrangian_t(const Variable& primal_dual_v, Variable& lag)
{

  using value_t = typename Variable::value_type;
  using array_t = bcp::buffer_adaptor<value_t>;

  auto n = variablesSize();
  auto m = constraintsSize();

  auto ptr = const_cast<value_t*>(primal_dual_v.data());

  array_t variables(n, ptr);
  array_t multipliers(m, ptr + n);
  array_t obj_factor(1, ptr + m + n); 
  array_t objective(lag.size(), lag.data());

  bool ret = this->evalObjective_t(variables, objective);
  lag[0] = obj_factor[0] * objective[0];

  if (ret)
  {
    std::vector<value_t> constraints(m);
    array_t g(m, constraints.data());
    ret = this->evalConstraints_t(variables, g);

    for (size_t i = 0; i < m; ++i)
      lag[0] += multipliers[i] * constraints[i];
  }
  return ret;

}


//
// dOCPCppAD.h ends here
