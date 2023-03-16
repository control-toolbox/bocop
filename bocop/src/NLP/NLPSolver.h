// NLPSolver.h
//

#pragma once

#include <map>
#include <string>

#include <NLP.h>
#include <functional>

/** ***************************************************************************
* \class NLPSolver
* \brief NLPSolver implements the interface to a NLP solver
*
* It is generic and virtual, implemented by NLPSolverIpopt.
* It compounds class NLP (the actual NLP problem to be solved) via the TNLP member of NLPSolverIpopt
* ****************************************************************************/
class NLPSolver
{
public:
    virtual ~NLPSolver(void) = default;

    virtual void setOptions(std::map<std::string, std::string> definition_map) = 0;
    virtual void setNLP(NLP *, const std::function<void(const std::vector<std::vector<double>>& ,
                                                        const std::vector<std::vector<double>>&)>& xUpdated  = [](const std::vector<std::vector<double>>& state, const std::vector<std::vector<double>>& control) {}) = 0; // note: actual NLP is in the TNLP member of NLPSolverIpopt
    virtual void solve(void) = 0;
 //   virtual void save(void) = 0;

};

//
// NLPSolver.h ends here
