3.1.2
  - added warmstart option to run() and solve(), ie initialize (state,control,params) from an existing solution file
  Note that both Runge Kutta internal variables (k_i) and multipliers are not taken into account.
  - build() and solve() now accept a 'cmake_options' argument (can be used to force g++ on binder)
  - discretization formulas: 'midpoint_implicit', 'euler_implicit', 'gauss2', 'gauss3'
  - average control (on steps) saved in .sol file and recovered by python readSolution()

3.1.1
  - python readSolution now reads the constants, boundary conditions and path constraints in the .sol file
  - remove ipopt.opt files: use instead keys in .def file (ipopt.<ipopt_key>) and pass the options to Ipopt in C++

3.1.0
  - added basic graphical interface 'bocopGUI' (using pysimpleGUIQt)
  - better handling/flushing of output during build and run steps (from command line, GUI and notebooks)
  - build step now handles debug and release modes
