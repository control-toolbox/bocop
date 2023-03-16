//  NLPSolverIpopt.h
//



#pragma once

#include <NLPSolver.h>
#include <IpIpoptApplication.hpp>
#include <IpTNLP.hpp>
#include <IpIpoptData.hpp>
#include <IpIpoptCalculatedQuantities.hpp>


/** ***************************************************************************
* \class NLPSolverIpopt
* \brief NLPSolverIpopt implements the interface to the Ipopt solver
*
* It derives from NLPSolver and contains a NLP member in its TNLP member
* It compounds Ipopt::TNLP and implements all callbacks from Ipopt using functions from NLPSolver and NLP
* ****************************************************************************/
class NLPSolverIpopt final : public NLPSolver
{
public:
     NLPSolverIpopt(void);
    ~NLPSolverIpopt(void);

    void setOptions(std::map<std::string, std::string> definition_map) override;
    void setNLP(NLP *bocopNLP,
                const std::function<void(const std::vector<std::vector<double>>& ,
                                         const std::vector<std::vector<double>>&)>& xUpdated = [](const std::vector<std::vector<double>>& state, const std::vector<std::vector<double>>& control) {}) override;
    void solve(void) override;

public:
    Ipopt::SmartPtr<Ipopt::TNLP> tnlp;
    Ipopt::SmartPtr<Ipopt::IpoptApplication> app;
};

// ///////////////////////////////////////////////////////////////////

inline NLPSolver *NLPSolverIpoptCreator(void)
{
    return new NLPSolverIpopt();
}

// +++ ?
class NLPSolverResults
{
public:
    int m_status = -1;
    double m_objective = -1;
    double m_constraint = -1;
    int m_iterations = -1;
};

//
// NLPSolverIpopt.h ends here
