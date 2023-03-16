#include <OCP.h>
#include <dOCPCppAD.h>
#include <NLPSolverIpopt.h>


/** ********************************************************************
* \mainpage Bocop3 main workflow
*
* create OCP and initialize from .def file
*
* create dOCP from OCP
*
* create NLPSolver (Ipopt)
*
* call solve() from NLPSolver
*
* **********************************************************************/

int main(int argc, char **argv)
{

    // +++ renvoyer des status plutot que des void pour les init

    // OCP definition and initialization
    OCP *myocp = new OCP();
    myocp->initialize();
    std::string definition_file;
    if (argc > 1)
        definition_file = argv[1];
    else
        definition_file = "problem.def";
    myocp->load(definition_file);

    // dOCP initialization
    dOCP *mydocp = new dOCPCppAD();
    mydocp->setOCP(myocp);
    mydocp->initialize();

    const std::function<void(const std::vector<std::vector<double>>& ,
                             const std::vector<std::vector<double>>&)>& dummyCallback  = [](const std::vector<std::vector<double>>& state, const std::vector<std::vector<double>>& control) {
                                                                                             std::cout<< "I am called !!!" << std::endl;
                                                                                         };

    // NLP solver initialization
    NLPSolver *mysolver = new NLPSolverIpopt();
    //mysolver->setNLP(mydocp, dummyCallback);
    mysolver->setNLP(mydocp);
    mysolver->setOptions(myocp->getDefinitionMap());

    // Solve problem and save solution
    // +++ later use a save() instead of saving at the end of solve() and pass sol name (.def prefix + .sol ?)
    mysolver->solve();

    // Clean
    delete myocp;
    delete mydocp;
    delete mysolver;

    // +++ retrieve status from solve()
    return 0;
}

//
// main.cpp ends here
