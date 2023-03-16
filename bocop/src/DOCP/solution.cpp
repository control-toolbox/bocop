#include <solution.h>


//+++ weird syntax, use list
dOCPsolution::dOCPsolution()
{
  init(0, 0, 0, 0, 0, 0, 0);
}

dOCPsolution::dOCPsolution( int dimState,
                            int dimSteps,
                            int dimControl,
                            int dimParameter,
                            int dimStages,
                            int dimBoundaryConditions,
                            int dimPathConstraints)
{
  init(dimState, dimSteps, dimControl, dimParameter, dimStages, dimBoundaryConditions, dimPathConstraints);
}

// +++ is this used at all ???
dOCPsolution::~dOCPsolution()
{
  // time grids
  delete this->time_steps_grid;
  delete this->time_stages_grid;
    
  // variables
  delete this->state;
  delete this->control;
  delete this->parameter;
  delete this->kstage;

  // multipliers
  delete this->boundary_conditions_multiplier;
  delete this->path_constraints_multiplier;
  delete this->adjoint_state;

  // constraints (bounds info should be retrievable from .def copy)
  delete this->boundary_conditions;
  delete this->path_constraints;
  delete this->dyn_equations;
}

void dOCPsolution::init(int dimState,
                        int dimSteps,
                        int dimControl,
                        int dimParameter,
                        int dimStages,
                        int dimBoundaryConditions,
                        int dimPathConstraints)
{
  // time
  this->time_steps_grid = new std::vector<double>();
  this->time_stages_grid = new std::vector<double>();

  // variables
  this->state = new std::vector<std::vector<double> >(dimState,std::vector<double>(dimSteps+1));
  this->control = new std::vector<std::vector<double> >(dimControl,std::vector<double>(dimSteps*dimStages));
  this->parameter = new std::vector<double>(dimParameter);
  this->kstage = new std::vector<std::vector<double> >(dimState,std::vector<double>(dimSteps*dimStages));

  // multipliers
  this->boundary_conditions_multiplier = new std::vector<double>(dimBoundaryConditions);
  this->path_constraints_multiplier = new std::vector<std::vector<double> >(dimPathConstraints,std::vector<double>(dimSteps));
  this->adjoint_state = new std::vector<std::vector<double> >(dimState,std::vector<double>(dimSteps));

  // constraints (bounds info should be retrievable from .def copy)
  this->boundary_conditions = new std::vector<double>(dimBoundaryConditions);
  this->path_constraints = new std::vector<std::vector<double> >(dimPathConstraints,std::vector<double>(dimSteps));
  this->dyn_equations = new std::vector<std::vector<double> >(dimState,std::vector<double>(dimSteps));

}


void dOCPsolution::readColumn(std::vector<double>& v, std::ifstream& in, std::string label)
{

  //while(!l.isEmpty() && !in.atEnd()) {
  //  v.push_back(l.toDouble());
  //  l = in.readLine();
  //}
}


bool dOCPsolution::readFile(const std::string file_path)
{
  std::ifstream infile(file_path);
  if (infile.is_open())
  {
    std::string line;
    while (std::getline(infile, line))
    {
    }
    std::cout << "dOCPsolution::readFile TO BE REDONE" << std::endl;
    exit(1);
  }
  else
    return false; // something went wrong.

}
