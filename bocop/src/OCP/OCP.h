// OCP.h
//

#pragma once

#include <map> 
#include <string>

#include <tools.h>

#include <cppad/cppad.hpp>

using double_ad = CppAD::AD<double>;

/** ***************************************************************************
* \class OCP
* \brief OCP defines an optimal control problem of the general form
* \f[
* \begin{array}{l}
* Min\ g_0(t_0,t_f,x(t_0),x(t_f),p,c)\\
* \dot x(t) = f(t,x,u,p,c)\\
* \phi_l \le \phi(t_0,t_f,x(t_0),x(t_f),p,c) \le \phi_u\\
* g_l \le g(t,x,u,p,c) \le g_u
* \end{array}
* \f]
* +++ state control param constant
*
* The actual functions are defined for each problem in a single cpp file
* Functions are defined using templates, for both basic double and ADdouble from CppAD
* since derivatives are required to solve the discretized problem.
* 
* Note that the class stores all parameters in a single dictionnary, which can include values
* not directly part of OCP but rather intended for use in classes using OCP objects
* (ex: time discretisation parameters for a direct transcription method, NLP solver  options, ...)
*
******************************************************************************/
class OCP
{

public:
  OCP(void);
  virtual ~OCP(void);
  
  virtual void initialize(void);
  void load(const std::string&);
  void save(std::ofstream& file_out);

  // handle definition file and map
  const std::string& getDefinitionForKey(const std::string& key, const std::string& default_value = "") const;
  int readDefinitionFile(const std::string& definition_file);

  /** \name getters */
  /**@{*/
  std::size_t stateSize() {return state_size;}
  std::size_t controlSize() {return control_size;}
  std::size_t parametersSize() {return parameters_size;}
  std::size_t constantsSize() {return constants_size;}
  std::size_t boundaryConditionsSize() {return boundary_conditions_size;}
  std::size_t pathConstraintsSize() {return path_constraints_size;}
  double initialTime() {return initial_time;}
  double finalTime() {return final_time;}
  std::vector<double> getConstants() {return constants;}
  //std::stringstream getFreeBounds() {return freebounds;}
  std::vector<double> stateLowerBounds() {return state_lower_bounds;}
  std::vector<double> stateUpperBounds() {return state_upper_bounds;}
  std::vector<double> controlLowerBounds() {return control_lower_bounds;}
  std::vector<double> controlUpperBounds() {return control_upper_bounds;}
  std::vector<double> paramLowerBounds() {return param_lower_bounds;}
  std::vector<double> paramUpperBounds() {return param_upper_bounds;}
  std::vector<double> boundaryLowerBounds() {return boundary_conditions_lower_bounds;}
  std::vector<double> boundaryUpperBounds() {return boundary_conditions_upper_bounds;}
  std::vector<double> pathLowerBounds() {return path_constraints_lower_bounds;}
  std::vector<double> pathUpperBounds() {return path_constraints_upper_bounds;}
  std::map<std::string, std::string> getDefinitionMap() {return definition_map;}
  /**@}*/

  /** \name setters */
  /**@{*/
  void setBounds(std::string prefix, const std::size_t dim, std::vector<double> &lower_bounds, std::vector<double> &upper_bounds);
  /**@}*/

  /** \name Optimal Control Problem specific functions (problem cpp file)
   * */
  /**@{*/
  /** \fn finalCost
   * Final cost \f$g_0\f$ (Mayer)
   * */
  template <typename Variable> void finalCost(double initial_time, double final_time, const Variable *initial_state, const Variable *final_state, const Variable *parameters, const double *constants, Variable &final_cost);
  /** \fn dynamics
   * State dynamics \f$f\f$
   * */
  template <typename Variable> void dynamics(double time, const Variable *state, const Variable *control, const Variable *parameters, const double *constants, Variable *state_dynamics);
  /** \fn boundaryConditions
   *  Boundary conditions \f$\phi\f$ at initial and final time
   * */
  template <typename Variable> void boundaryConditions(double initial_time, double final_time, const Variable *initial_state, const Variable *final_state, const Variable *parameters, const double *constants, Variable *boundary_conditions);
  /** \fn pathConstraints
   * Path constraints \f$g\f$ (includes both mixed and pure state constraints)
   * */
  template <typename Variable> void pathConstraints(double time, const Variable *state, const Variable *control, const Variable *parameters, const double *constants, Variable *path_constraints);
  /** \fn preProcessing
   * Preprocessing operations (performed once before the optimisation)
   * WARNING: CURRENTLY CALLED AT OCP initialize(), NAMELY *BEFORE* READING THE PROBLEM .DEF FILE
   * CONSTANTS IN PARTICULAR ARE NOT AVAILABLE -_-
   * */
  void preProcessing(void);
  /**@}*/

  std::stringstream freebounds; //getter causes error deleted function -_-

private:

  // dimensions
  std::size_t state_size = 0;
  std::size_t control_size = 0;
  std::size_t parameters_size = 0;
  std::size_t constants_size = 0;
  std::size_t boundary_conditions_size = 0;
  std::size_t path_constraints_size = 0;

  // initial and final time
  double initial_time = 0;
  double final_time = 0;

  // constants
  std::vector<double> constants;

  // bounds for variables and constraints

  std::vector<double> state_lower_bounds;
  std::vector<double> state_upper_bounds;
  std::vector<double> control_lower_bounds;
  std::vector<double> control_upper_bounds;
  std::vector<double> param_lower_bounds;
  std::vector<double> param_upper_bounds;
  std::vector<double> boundary_conditions_lower_bounds;
  std::vector<double> boundary_conditions_upper_bounds;
  std::vector<double> path_constraints_lower_bounds;
  std::vector<double> path_constraints_upper_bounds;

  // dictionary of (key,value) from .def file
  std::map<std::string, std::string> definition_map;

};

// ///////////////////////////////////////////////////////////////////




// ///////////////////////////////////////////////////////////////////
// python wrap
/*
#ifndef SWIGPYTHON

template <class D>
class OCPBase : public OCP
{
public:
  void finalCost_impl(double initial_time, double final_time, const double *initial_state, const double *final_state, const double *parameters, const double *constants, double &final_cost) final
  { static_cast<D *>(this)->finalCost(initial_time, final_time, initial_state, final_state, parameters, constants, final_cost); };

  void dynamics_impl(double time, const double *state, const double *control, const double *parameters, const double *constants, double *state_dynamics) final
  { static_cast<D *>(this)->dynamics(time, state, control, parameters, constants, state_dynamics); };

  void boundaryConditions_impl(double initial_time, double final_time, const double *initial_state, const double *final_state, const double *parameters, const double *constants, double *boundary_conditions) final
  { static_cast<D *>(this)->boundaryConditions(initial_time, final_time, initial_state, final_state, parameters, constants, boundary_conditions); };

  void pathConstraints_impl(double time, const double *state, const double *control, const double *parameters, const double *constants, double *path_constraints) final
  { static_cast<D *>(this)->pathConstraints(time, state, control, parameters, constants, path_constraints); };

};

#endif
*/

//
// OCP.h ends here
