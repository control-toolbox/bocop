// dOCPCppAD.cpp
//

#include <dOCPCppAD.h>
#include <tools.h>
template <typename T> using view_t = bcp::buffer_adaptor<T>;


#include <vector>



// ///////////////////////////////////////////////////////////////////
// dOCPCppAD implementation
// ///////////////////////////////////////////////////////////////////


dOCPCppAD::dOCPCppAD(void) : dOCP()
{}

dOCPCppAD::~dOCPCppAD(void)
{}

void dOCPCppAD::initialize(void)
{
  dOCP::initialize();

  ad_retape = stoi(ocp->getDefinitionForKey("ad.retape","0"));

  // initialise the derivatives (to be called again at each evaluation in case of retaping)
  std::cout << "Initializing derivatives (CppAD): Gradient ... ";
  std::cout.flush();
  initializeObjectiveGradient(starting_point.data());
  std::cout << "Jacobian ... "; 
  std::cout.flush();
  initializeConstraintsJacobian(starting_point.data());
  std::cout << "Hessian ... ";
  std::cout.flush();    
  initializeLagrangianHessian(starting_point.data());
  std::cout << "Done" << std::endl;  

}


// OBJECTIVE, CONSTRAINTS, LAGRANGIAN
bool dOCPCppAD::evalObjective(const double *variables, double &objective)
{
  if (ad_retape == 1)
  {
    //std::cout << "retaping objective" << std::endl;
    initializeObjectiveGradient(variables);
  }

  using array_t = bcp::buffer_adaptor<double>;
  array_t var(variablesSize(), const_cast<double*>(variables));
  array_t obj(1, &objective);

  evalObjective_t(var, obj);

  return true;
}

bool dOCPCppAD::evalConstraints(const double *variables, double *constraints)
{
  if (ad_retape == 1)
  {
    //std::cout << "retaping constraints" << std::endl;
    initializeConstraintsJacobian(variables);
  }

  using array_t = bcp::buffer_adaptor<double>;
  array_t v(this->variablesSize(), const_cast<double*>(variables));
  array_t g(this->constraintsSize(), constraints);

  evalConstraints_t(v, g);

  return true;
}

bool dOCPCppAD::evalLagrangian(const double *primal_dual_variables, double &lagrangian)
{
  if (ad_retape == 1)
  {
    //std::cout << "retaping lagrangian" << std::endl;
    initializeLagrangianHessian(primal_dual_variables);
  }

  using array_t = bcp::buffer_adaptor<double>;
  array_t p_d_v(this->variablesSize()+this->constraintsSize()+1, const_cast<double*>(primal_dual_variables));
  array_t lag(1, &lagrangian);

  //+++ obj_factor ?

  return evalLagrangian_t(p_d_v, lag);
}


// OBJECTIVE GRADIENT

void dOCPCppAD::initializeObjectiveGradient(const double *x)
{
  //std::cout << "initialize gradient" << std::endl;
  auto n = this->variablesSize();
  x_ad.resize(n);
  for (size_t i=0; i<n; i++)
    x_ad[i] = x[i];
  objvalue_ad.resize(1);
  CppAD::Independent(x_ad);
  evalObjective_t(x_ad, objvalue_ad);
  f_obj.Dependent(x_ad, objvalue_ad);
  grad.resize(n);
  if (ad_retape == 0)
    f_obj.optimize();
}

bool dOCPCppAD::evalObjectiveGradient(const double *variables, double *grad_objective)
{
  auto ptr = variables;
  auto n = this->variablesSize();
  // use jacobian with objective as a size 1 vector
  std::copy(ptr, ptr + n, x2.begin());
  grad = f_obj.Jacobian(x2);
  std::copy(grad.begin(), grad.end(), grad_objective);

  return true;
}



//***********************************************************************************************
// CONSTRAINTS JACOBIAN
//***********************************************************************************************

void dOCPCppAD::initializeConstraintsJacobian(const double *x)
{

  //std::cout << "initialize jacobian" << std::endl;
  auto n = variablesSize();
  auto m = constraintsSize();
  g_ad.resize(m);
  x_ad.resize(n);
  for (size_t i=0; i<n; i++)
    x_ad[i] = x[i];
  CppAD::Independent(x_ad);
  evalConstraints_t(x_ad, g_ad);
  g_con.Dependent(x_ad, g_ad);
  if (ad_retape == 0)
    g_con.optimize();

  // initialize sparse structure for jacobian
  auto dim = std::min(n,m);
  vector_ad<std::set<size_t> > r(dim);
  for (std::size_t i = 0; i < dim; ++i)
    r[i].insert(i);

  if (n <= m)
    pattern_jac = g_con.ForSparseJac(n, r);
  else
    pattern_jac = g_con.RevSparseJac(m, r);

  // retrieve index sets and nnz for jacobian
  std::size_t j;
  row_jac.clear();
  col_jac.clear();
  for (std::size_t i = 0; i < m; ++i)
  {
    auto itr = pattern_jac[i].begin();
    auto end = pattern_jac[i].end();
    while (itr != end)
    {
      j = *itr++;
      row_jac.push_back(i);
      col_jac.push_back(j);
    }
  }
  jac.resize(col_jac.size());
  x2.resize(n);
}


bool dOCPCppAD::setConstraintsJacobianSparsityPattern(int *row_indices, int *col_indices, int jacobian_nonzero)
{
  bool res;
  int nnz = (int) jacobianNonZeroEntries();
  if (nnz != jacobian_nonzero)
    res = false;
  else
  {
    auto row_ids = jacobianRowIndices();
    auto col_ids = jacobianColIndices();
    for (int idx = 0; idx < nnz; idx++)
    {
      row_indices[idx] = (int) row_ids[idx] + 1;
      col_indices[idx] = (int) col_ids[idx] + 1;
    }
    res = true;
  }
  return res;
}


bool dOCPCppAD::evalConstraintsJacobian(const double *variables, double *jac_constraints)
{

  if (jac_constraints)
  {
    auto ptr = variables;
    auto n = variablesSize();
    auto m = constraintsSize();
    std::copy(ptr, ptr + n, x2.begin());

    //use forward or reverse mode depending on dimensions
    if (n <= m)
      g_con.SparseJacobianForward(x2, pattern_jac, row_jac, col_jac, jac, work_jac);
    else
      g_con.SparseJacobianReverse(x2, pattern_jac, row_jac, col_jac, jac, work_jac);

    std::copy(jac.begin(), jac.end(), jac_constraints);

    return true;
  }
  return false;
}



//***********************************************************************************************
// LAGRANGIAN HESSIAN
//***********************************************************************************************

void dOCPCppAD::initializeLagrangianHessian(const double *x)
{
  //std::cout << "initialize hessian" << std::endl;

  // FULL HESSIAN VERSION +++ todo reduced hessian wrt x only
  // init
  auto n = variablesSize();
  auto m = constraintsSize();
  std::vector<double_ad> xlf_ad(n+m+1, 1.0);
  for (size_t i=0; i<n; i++)
    xlf_ad[i] = x[i];
  lagvalue_ad.resize(1);

  // Lagrangian as function of (x,lambda,objfactor)
  CppAD::Independent(xlf_ad);
  evalLagrangian_t(xlf_ad, lagvalue_ad);
  h_lag.Dependent(xlf_ad, lagvalue_ad);
  if (ad_retape == 0)
    h_lag.optimize();

  // initialize sparse structure for hessian
  std::size_t dim = n+m+1;
  vector_ad<std::set<std::size_t>> r_set(dim);
  for (std::size_t i = 0; i < dim; ++i)
    r_set[i].insert(i);
  h_lag.ForSparseJac(dim, r_set);

  vector_ad<std::set<std::size_t>> s_set(1);
  s_set[0].insert(0);
  pattern_hess = h_lag.RevSparseHes(dim, s_set);

  // retrieve index sets and nnz for hessian
  row_hess.clear();
  col_hess.clear();
  std::size_t j;
  for (std::size_t i = 0; i < dim; ++i)
  {
    auto itr = pattern_hess[i].begin();
    auto end = pattern_hess[i].end();
    while (itr != end)
    {
      j = *itr++;
      // keep only the upper triangular n x n part of the hessian
      if (j<n && i<=j)
      {
        row_hess.push_back(i);
        col_hess.push_back(j);
      }
    }
  }
  hess.resize(col_hess.size());
  xlf2.resize(dim);
}

bool dOCPCppAD::setLagrangianHessianSparsityPattern(int *row_indices, int *col_indices, int hessian_nonzero)
{
  bool res;
  int nnz = (int) hessianNonZeroEntries();
  if (nnz != hessian_nonzero)
    res = false;
  else
  {
    auto row_ids = hessianRowIndices();
    auto col_ids = hessianColIndices();
    for (int idx = 0; idx < nnz; idx++)
    {
      row_indices[idx] = (int) row_ids[idx] + 1;
      col_indices[idx] = (int) col_ids[idx] + 1;
    }
    res = true;
  }
  return res;
}

bool dOCPCppAD::evalLagrangianHessian(const double *variables, double obj_factor, const double *lambda, double *lag_hessian)
{
  if (lag_hessian)
  {
    auto n = variablesSize();
    auto m = constraintsSize();
    auto nnz = hessianNonZeroEntries();

    using array_t = bcp::buffer_adaptor<double>;

    array_t x(n, const_cast<double *>(variables));
    array_t l(m, const_cast<double *>(lambda));
    array_t h(nnz, lag_hessian);

    std::vector<double> w(1, 1.0);

    std::copy(x.begin(), x.end(), xlf2.begin());
    std::copy(l.begin(), l.end(), xlf2.begin()+n);
    xlf2[n+m] = obj_factor;

    // compute the n x n upper submatrix only (cf building row_hess and col_hess)
    h_lag.SparseHessian(xlf2, w, pattern_hess, row_hess, col_hess, hess, work_hess);

    std::copy(hess.begin(), hess.end(), h.begin());

    return true;
  }

  return false;
}

//
// dOCPCppAD.cpp ends here
