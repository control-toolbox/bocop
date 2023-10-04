#pragma once

%module(directors="1") bocopwrapper

#ifdef SWIGWIN
%include <windows.i>
#endif

%include <std_vector.i>
%include <std_string.i>
%template(VecDouble) std::vector<double>;
%template(VecVecdouble) std::vector< std::vector<double> >;

%{
#define SWIG_FILE_WITH_INIT
#include <vector>
#include <OCP.h>
#include <dOCPCppAD.h>
#include <NLPSolverIpopt.h>
%}

// /////////////////////////////////////////////////////////////////
// Preprocessing setup
// /////////////////////////////////////////////////////////////////

#pragma SWIG nowarn=302, 315, 389, 401, 509, 801, 472, 473, 476, 362, 503, 514, 516, 842, 845

#undef  BOCOPWRAPPER_EXPORT
#define BOCOPWRAPPER_EXPORT

// virtual wrapper
%feature("director") CallbackF;
%inline %{
struct CallbackF 
{
    virtual void handle(const std::vector<std::vector<double>>& state,
                        const std::vector<std::vector<double>>& control) const {};
  virtual std::string def_file_path() const {return "./problem.def";};
  virtual std::string sol_file_path() const {return "./problem.sol";};
 virtual ~CallbackF() {}
};

 %}


%{
static CallbackF *handler_ptr = NULL;
static void handler_helper(const std::vector<std::vector<double>>& state,
                           const std::vector<std::vector<double>>& control) 
{
  // Make the call up to the target language when handler_ptr
  // is an instance of a target language director class
    return handler_ptr->handle(state, control);
}
%}

%inline %{
void solve(CallbackF *callback = nullptr, std::string def_file = "./problem.def", std::string sol_file = "./problem.sol") 
{
  handler_ptr = callback;

  // OCP definition and initialization
  OCP *myocp = new OCP();
  myocp->initialize();
  if (callback)
    myocp->load(callback->def_file_path()); 
  else
    myocp->load(def_file); //+++ pass definition file name instead

  // dOCP initialization
  dOCP *mydocp = new dOCPCppAD();
  mydocp->setOCP(myocp);
  mydocp->initialize();
  if (callback)
    mydocp->solution_file = callback->sol_file_path();    
  else
    mydocp->solution_file = sol_file;

  // NLP solver initialization
  NLPSolver *mysolver = new NLPSolverIpopt();
  if(callback)
      mysolver->setNLP(mydocp, &handler_helper);
  else
      mysolver->setNLP(mydocp);
  mysolver->setOptions(myocp->getDefinitionMap());

  // Solve problem and save solution
  mysolver->solve();
  
  // Clean
  delete myocp;
  delete mydocp;
  delete mysolver;
  callback = NULL;
}
%}



%pythoncode %{
import numpy as np
import matplotlib.pyplot as plt
from IPython.display import display, clear_output
import ipywidgets as widgets

class PyCallback(CallbackF):
    def __init__(self):
        super().__init__()
        self.initialized = False
        self.g_state = []         #for each iter, 2 dims np array
        self.g_control = []
        self.out = widgets.Output()
        self.outText = widgets.Output(layout={'border': '1px solid black'})

    def update(self, iteration):
        if self.initialized:
            len_state = len(self.g_state[iteration])
            clear_output(wait=True)
            for i in range(len_state):
                self.axs[i].clear()
                self.axs[i].plot(self.x_state ,self.g_state[iteration][i])
                self.axs[i].title.set_text(f"State {i}")
                self.axs[i].relim()
                self.axs[i].autoscale()
                self.fig.canvas.blit(self.axs[i].bbox)

            for i in range(len(self.g_control[iteration])):
                self.axs[len_state + i].clear()
                self.axs[len_state + i].plot(self.x_control ,self.g_control[iteration][i], color='r')
                self.axs[len_state + i].relim()
                self.axs[len_state + i].set_title(f"Control {i}")
                self.axs[len_state + i].autoscale()
                self.fig.canvas.blit(self.axs[len_state + i].bbox)

            self.outText.clear_output()
            with self.outText:
                objective = self.g_state[iteration][-1][-1]
                print(f"iteration {iteration}  objective : {objective}")

            display(self.fig)
            display(self.outText)

    def handle(self, state, control):
        if not self.initialized:
            self.initialized = True
            plt.ioff()
            nrows = len(state) + len(control)
            with self.out:
                self.fig, self.axs = plt.subplots(nrows=nrows, ncols=1)
                self.fig.tight_layout()
                self.fig.canvas.header_visible = False # Hide the Figure name at the top of the figure
                # If true then scrolling while the mouse is over the canvas will not move the entire notebook
                self.fig.canvas.capture_scroll = True

            self.x_state = np.arange(len(state[0]))
            self.x_control = np.arange(len(control[0]))
            plt.ion()

        self.g_state.append(np.asarray(state))
        self.g_control.append(np.asarray(control))
        self.update(len(self.g_state) -1)

 %}

// /////////////////////////////////////////////////////////////////
// Input
// /////////////////////////////////////////////////////////////////
