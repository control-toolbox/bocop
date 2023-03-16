// NLP.h
//

#pragma once

#include <vector>
#include <string>


/** ***************************************************************************
* \class NLP
* \brief NLP defines a nonlinear programming problem of the general form
* \f[
* \begin{array}{l}
* Min\ F(X)\\
* G_L \le G(X) \le G_U
* \end{array}
* \f]
* It provides functions for objective F and constraints C (and Lagrangian L)
* with their derivatives gradient_F, jacobian_G and hessian_L
* as well as some utility functions (set bounds, starting point,...)
*
* Note: the class is virtual and its functions are typically overriden in dOCP and dOCPCppAD
* ****************************************************************************/
class NLP
{
public:
  virtual ~NLP(void) = default;

  virtual void initialize(void) = 0;

  // NLP solution
  virtual std::string solutionFile(void) const = 0;
  virtual void setSolutionFile(const std::string solFile) = 0;
  virtual void writeSolution(const int status, const int iter, const double objective, const double constraints_viol, const double *variables, const double *multipliers, const double *constraints) = 0;

  // NLP variables and bounds
  virtual std::size_t variablesSize(void) const = 0;
  virtual std::size_t constraintsSize(void) const = 0;
  virtual std::vector<double> startingPoint(void) const = 0;
  virtual std::vector<double> variablesLowerBounds(void) const = 0;
  virtual std::vector<double> variablesUpperBounds(void) const = 0;
  virtual std::vector<double> constraintsLowerBounds(void) const = 0;
  virtual std::vector<double> constraintsUpperBounds(void) const = 0;

  /** \name  Functions and derivatives for the NLP solver */
  /**@{*/
  /** Objective function for the nonlinear optimization problem */
  virtual bool evalObjective(const double *variables, double& objective) = 0;

  /** Constraints for the nonlinear problem. Equality constraints are handled by setting equal values for lower and upper bound. */
  virtual bool evalConstraints(const double *variables, double *constraints) = 0;

  /** Lagrangian \f$L(X,\lambda) = F(X) + \lambda . C(X)\f$. This function is not called directly but its Hessian is required by the NLP solver.*/
  virtual bool evalLagrangian(const double *primal_dual_variables, double& lagrangian) = 0;

  /** Gradient of the objective, computed by automatic differentiation (CppAD) */
  virtual bool evalObjectiveGradient(const double *variables, double *grad_objective) = 0;

  /** Jacobian of the constraints, computed by automatic differentiation (CppAD) in sparse format. */
  virtual bool evalConstraintsJacobian(const double *variables, double *jac_constraints) = 0;

  /** Hessian of the Lagrangian, computed by automatic differentiation (CppAD) in sparse format.
   * Note: currently Hessian is computed fully ie for both primal and dual variables, while only the derivatives wrt X are used.
   * Todo: use cppad DYNAMIC type for the dual variables.
   * */
  virtual bool evalLagrangianHessian(const double *variables, double obj_factor, const double *lambda, double *lag_hessian) = 0;
  /**@}*/

  virtual std::size_t jacobianNonZeroEntries(void) const = 0;
  virtual const std::size_t *jacobianRowIndices(void) const = 0;
  virtual const std::size_t *jacobianColIndices(void) const = 0;
  virtual bool setConstraintsJacobianSparsityPattern(int *row_indices, int *col_indices, int jacobian_nonzero) = 0;

  virtual std::size_t hessianNonZeroEntries(void) const = 0;
  virtual const std::size_t *hessianRowIndices(void) const = 0;
  virtual const std::size_t *hessianColIndices(void) const = 0;
  virtual bool setLagrangianHessianSparsityPattern(int *row_indices, int *col_indices, int hessian_nonzero) = 0;
};

//
// NLP.h ends here
