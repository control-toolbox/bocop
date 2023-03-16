// NLPSolverIpopt
//

#include <NLPSolverIpopt.h>
#include <dOCP.h>
#include <functional>

#include <IpIpoptCalculatedQuantities.hpp>
#include <IpIpoptData.hpp>
#include <IpTNLPAdapter.hpp>
#include <IpOrigIpoptNLP.hpp>

// ///////////////////////////////////////////////////////////////////
// TNLP implementation
// ///////////////////////////////////////////////////////////////////

class TNLPImpl : public Ipopt::TNLP
{

public:
  // initialisations
  bool get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g, Ipopt::Index& nnz_h_lag, IndexStyleEnum& Index_style) override;
  bool get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u, Ipopt::Index   m, Ipopt::Number* g_l, Ipopt::Number* g_u) override;
  bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number* x, bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U, Ipopt::Index m, bool init_lambda, Ipopt::Number* lambda) override;

  bool intermediate_callback(Ipopt::AlgorithmMode mode,
                             Ipopt::Index iter, Ipopt::Number obj_value,
                             Ipopt::Number inf_pr, Ipopt::Number inf_du,
                             Ipopt::Number mu, Ipopt::Number d_norm,
                             Ipopt::Number regularization_size,
                             Ipopt::Number alpha_du, Ipopt::Number alpha_pr,
                             Ipopt::Index ls_trials,
                             const Ipopt::IpoptData* ip_data,
                             Ipopt::IpoptCalculatedQuantities* ip_cq) override;


  // post optimisation call
  void finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n, const Ipopt::Number* x, const Ipopt::Number* z_L, const Ipopt::Number* z_U, Ipopt::Index m, const Ipopt::Number* g,
                         const Ipopt::Number* lambda, Ipopt::Number obj_value, const Ipopt::IpoptData* ip_data, Ipopt::IpoptCalculatedQuantities* ip_cq);

  // objective
  bool eval_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number& obj_value) override;
  bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number* grad_f) override;

  // constraints
  bool eval_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Index m, Ipopt::Number* g) override;
  bool eval_jac_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow, Ipopt::Index* jCol, Ipopt::Number* values) override;

  // lagrangian
  bool eval_h(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number* lambda, bool new_lambda,
              Ipopt::Index nele_hess, Ipopt::Index* iRow, Ipopt::Index* jCol, Ipopt::Number* values) override;

public:
  NLP *bocopNLP = nullptr;
  std::function<void(const std::vector<std::vector<double>>& , const std::vector<std::vector<double>>&)> xUpdated;

protected:
  friend class NLPSolverIpopt;
  std::vector<std::vector<double> > m_state;
  std::vector<std::vector<double> > m_control;
};

// ///////////////////////////////////////////////////////////////////

bool TNLPImpl::get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g, Ipopt::Index& nnz_h_lag, IndexStyleEnum& Index_style)
{
  n = (int) bocopNLP->variablesSize();
  m = (int) bocopNLP->constraintsSize();

  nnz_jac_g = bocopNLP->jacobianNonZeroEntries();
  nnz_h_lag = bocopNLP->hessianNonZeroEntries();

  Index_style = TNLP::FORTRAN_STYLE;
  return true;
}

bool TNLPImpl::get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u, Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u)
{
  auto v_l = bocopNLP->variablesLowerBounds();
  std::copy(v_l.begin(), v_l.end(), x_l);
  auto v_u = bocopNLP->variablesUpperBounds();
  std::copy(v_u.begin(), v_u.end(), x_u);

  auto c_l = bocopNLP->constraintsLowerBounds();
  std::copy(c_l.begin(), c_l.end(), g_l);
  auto c_u = bocopNLP->constraintsUpperBounds();
  std::copy(c_u.begin(), c_u.end(), g_u);

  return true;
}

bool TNLPImpl::get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number* x, bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U,
                                  Ipopt::Index m, bool init_lambda, Ipopt::Number* lambda)
{
  if (init_x)
  {
    auto start = bocopNLP->startingPoint();
    std::copy(start.begin(), start.end(), x);
  }
  if (init_z || init_lambda)
  {
    std::cout << "asking for lambda or z init..." << std::endl;
    return false;
  }


  return true;
}


bool TNLPImpl::intermediate_callback(Ipopt::AlgorithmMode mode,
                                     Ipopt::Index iter, Ipopt::Number obj_value,
                                     Ipopt::Number inf_pr, Ipopt::Number inf_du,
                                     Ipopt::Number mu, Ipopt::Number d_norm,
                                     Ipopt::Number regularization_size,
                                     Ipopt::Number alpha_du, Ipopt::Number alpha_pr,
                                     Ipopt::Index ls_trials,
                                     const Ipopt::IpoptData* ip_data,
                                     Ipopt::IpoptCalculatedQuantities* ip_cq)
{
    //n = this->m_dimX   the number of variables x in the problem; it will have the same value that was specified in TNLP::get_nlp_info
    //m = this->m_dimConstraints  the number of constraints g(x) in the problem; it will have the same value that was specified in TNLP::get_nlp_info

    Ipopt::TNLPAdapter* tnlp_adapter = NULL;
    if( ip_cq != NULL )  {
        Ipopt::OrigIpoptNLP* orignlp;
        orignlp = dynamic_cast<Ipopt::OrigIpoptNLP*>(GetRawPtr(ip_cq->GetIpoptNLP()));
        if( orignlp != NULL ) {
            tnlp_adapter = dynamic_cast<Ipopt::TNLPAdapter*>(GetRawPtr(orignlp->nlp()));
            dOCP *docp = dynamic_cast<dOCP*>(bocopNLP);
            if(tnlp_adapter && docp) {
                int m_dimX = (int) docp->variablesSize();
                int dimState = docp->ocp->stateSize();
                int dimControl = docp->ocp->controlSize();
                int dimSteps = docp->discretisationSteps();
                int dimStages = docp->RKStages();

                double* primals = new double[m_dimX];
                tnlp_adapter->ResortX(*ip_data->curr()->x(), primals);

                docp->xd->getState(primals,
                                   docp->variables_offset_state,
                                   docp->discretisation_steps,
                                   dimState,
                                   m_state);

                docp->ud->getControl(primals,
                                     docp->variables_offset_control,
                                     dimSteps,
                                     dimStages,
                                     dimControl,
                                     m_control);

                delete[] primals;
                xUpdated(m_state, m_control);
            }
        }
    }
    return true;
}

bool TNLPImpl::eval_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number& obj_value)
{
  return bocopNLP->evalObjective(x, obj_value);
}

bool TNLPImpl::eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number* grad_f)
{
  return bocopNLP->evalObjectiveGradient(x, grad_f);
}

bool TNLPImpl::eval_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Index m, Ipopt::Number* g)
{
  return bocopNLP->evalConstraints(x, g);
}

bool TNLPImpl::eval_jac_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Index m,
                          Ipopt::Index nele_jac, Ipopt::Index* iRow, Ipopt::Index* jCol, Ipopt::Number* values)
{
  bool res;
  if (!values)
    // In this case Ipopt retrieves the sparse structure of the jacobian.
    res = bocopNLP->setConstraintsJacobianSparsityPattern(iRow, jCol, nele_jac);
  else
    res = bocopNLP->evalConstraintsJacobian(x, values);

  return res;
}

bool TNLPImpl::eval_h(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number* lambda,
                      bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index* iRow, Ipopt::Index* jCol, Ipopt::Number* values)
{
  bool res;
  if (!values)
    // In this case Ipopt retrieves the sparse structure of the hessian.
    res = bocopNLP->setLagrangianHessianSparsityPattern(iRow, jCol, nele_hess);
  else
    res = bocopNLP->evalLagrangianHessian(x, obj_factor, lambda, values);

  return res;
}

void TNLPImpl::finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n, const Ipopt::Number* x, const Ipopt::Number* z_L, const Ipopt::Number* z_U, Ipopt::Index m, const Ipopt::Number* g,
                                 const Ipopt::Number* lambda, Ipopt::Number obj_value, const Ipopt::IpoptData* ip_data, Ipopt::IpoptCalculatedQuantities* ip_cq)
{

  // retrieve solution data
  // +++ NB use a class for the NLP solution (maybe reuse/expand the existing one ?).
  // set it here, then writesolution can be called from NLP::save() in a more generic way
  int status_int = (int) status;
  int iter = ip_data->iter_count();
  double constraints_viol = (double)ip_cq->curr_nlp_constraint_violation(Ipopt::NORM_2);

  // save solution (NB reuse this for intermediate callback to save iterations !)
  if (!bocopNLP->solutionFile().empty())
    bocopNLP->writeSolution(status_int, iter, obj_value, constraints_viol, x, lambda, g);
}



// ///////////////////////////////////////////////////////////////////
// NLPSolverIpopt implementation
// ///////////////////////////////////////////////////////////////////

NLPSolverIpopt::NLPSolverIpopt(void) : NLPSolver()
{
  //app = IpoptApplicationFactory(); Factory does not seem available in windows conda package (older version ?)
  app = new Ipopt::IpoptApplication();

  // default base options
  app->Options()->SetIntegerValue("max_iter", 1000);
  app->Options()->SetIntegerValue("print_level", 5);
  app->Options()->SetNumericValue("tol", 1e-12);
  app->Options()->SetStringValue("mu_strategy", "adaptive");
}

NLPSolverIpopt::~NLPSolverIpopt(void)
{}

void NLPSolverIpopt::setOptions(std::map<std::string, std::string> definition_map)
{
  // loop on options map: retrieve and set ipopt keys
  for (std::map<std::string,std::string>::iterator it=definition_map.begin(); it!=definition_map.end(); ++it)
  {
    std::string key = it->first;
    if (key.rfind("ipoptIntOption", 0) == 0)
    {
      key.erase(0, key.find('.') + 1);
      std::cout << "Ipopt option: " << key << " => " << it->second << std::endl;
      app->Options()->SetIntegerValue(key, stoi(it->second));
    }
    else if (key.rfind("ipoptNumOption", 0) == 0)
    {
      key.erase(0, key.find('.') + 1);
      std::cout << "Ipopt option: " << key << " => " << it->second << std::endl;
      app->Options()->SetNumericValue(key, stod(it->second));
    }
    else if (key.rfind("ipoptStrOption", 0) == 0)
    {
      key.erase(0, key.find('.') + 1);
      std::cout << "Ipopt option: " << key << " => " << it->second << std::endl;
      app->Options()->SetStringValue(key, it->second);
    }
  }
}

void NLPSolverIpopt::setNLP(NLP *bocopNLP,
                            const std::function<void(const std::vector<std::vector<double>>& ,
                                                     const std::vector<std::vector<double>>&)>& callback)
{
  auto tnlp_impl = new TNLPImpl;
  tnlp_impl->bocopNLP = bocopNLP;
  tnlp_impl->xUpdated = callback;
  tnlp = tnlp_impl;

  //init vectors state and controls for the callback
  dOCP *docp = dynamic_cast<dOCP*>(bocopNLP);
  if(docp) {
      int dimState = docp->ocp->stateSize();
      int dimControl = docp->ocp->controlSize();
      int dimSteps = docp->discretisationSteps();
      int dimStages = docp->RKStages();

      //  m_state = new std::vector<std::vector<double> >(dimState,std::vector<double>(dimSteps+1))
      tnlp_impl->m_state.resize(dimState);
      for(auto&& v : tnlp_impl->m_state) {
          v.resize(dimSteps+1);
      }

      //m_control = new std::vector<std::vector<double> >(dimControl,std::vector<double>(dimSteps*dimStages));
      tnlp_impl->m_control.resize(dimControl);
      for(auto&& v: tnlp_impl->m_control) {
          v.resize(dimSteps*dimStages);
      }
  }
}

void NLPSolverIpopt::solve(void)
{
  Ipopt::ApplicationReturnStatus status;
  status = app->Initialize();
  std::cout.flush();
  if (status != Ipopt::Solve_Succeeded)
  {
    std::cout << "Error during Ipopt initialisation: " << status << std::endl;
    exit(1);
  }

  app->OptimizeTNLP(tnlp);
}

//
// NLPSolverIpopt.cpp ends here
