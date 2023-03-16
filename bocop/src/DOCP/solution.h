#include <vector>
#include <string>
#include <fstream>
#include <iostream>

//+++ test part should be removed (at least splitted to tools)


#define SOLVER_STATUS_TITLE "Solver status: "
#define SOLVER_ITERATIONS_TITLE "Solver iterations: "
#define OBJECTIVE_FUNCTION_TITLE "Objective function: "
#define CONSTRAINT_VIOLATION_TITLE "Constraints violation: "

#define TIME_STEPS_TITLE "# Time steps"
#define TIME_STAGES_TITLE "# Time stages"
#define STATE_TITLE "# State"
#define CONTROL_TITLE "# Control"
#define PARAMETER_TITLE "# Parameters"
#define MULTI_BOUNDARY_CONDITION_TITLE "# Multipliers associated to the boundary conditions"
#define MULTI_PATH_CONSTRAINTS_TITLE "# Multipliers associated to the path constraints"
#define ADJOINT_STATE_TITLE "# Adjoint state"
#define BOUNDARY_TITLE "# Boundary Conditions"
#define PATH_CONSTRAINTS_TITLE "# Path constraints"

class dOCPsolution
{
public:
    int status = -1;
    int iterations = -1;
    double objective = -1;
    double constraint = -1;

public:
    // time
    std::vector<double> *time_steps_grid = nullptr;
    std::vector<double> *time_stages_grid = nullptr;

    // variables
    std::vector<std::vector<double> > *state = nullptr;
    std::vector<std::vector<double> > *control = nullptr;
    std::vector<double> *parameter = nullptr;
    std::vector<std::vector<double> > *kstage = nullptr;

    // multipliers
    std::vector<double> *boundary_conditions_multiplier = nullptr;
    std::vector<std::vector<double> > *path_constraints_multiplier = nullptr;
    std::vector<std::vector<double> > *adjoint_state = nullptr;

    // constraints (bounds info should be retrievable from .def copy)
    std::vector<double> *boundary_conditions = nullptr;
    std::vector<std::vector<double> > *path_constraints = nullptr;
    std::vector<std::vector<double> > *dyn_equations = nullptr;

public:
    dOCPsolution(int, int, int, int, int, int, int);
    dOCPsolution(); // built with unitary size
    ~dOCPsolution();
    void initSize(int, int, int, int, int, int, int);
    bool readFile(const std::string file_path);
    void readColumn(std::vector<double>& v, std::ifstream& in, std::string label);


private:
    void init(int, int, int, int, int, int, int);
};
