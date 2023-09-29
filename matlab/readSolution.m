% Matlab function to read bocop .sol file
% Pierre Martinon
% Inria and CMAP Ecole Polytechnique
% 2015-2016

function [time_steps, time_stages, state, control, parameters,...
    boundarycond, pathcond,...
    boundarycond_multipliers, pathcond_multipliers, costate,... 
    status, objective, constraints, iterations] = readSolution(filename)


  %+++ probably better to read whole file then extract parts with subfunctions
  %+++ with current getters reads need to be done in the correct order ...
  solfile = fopen(filename);

  % read dimensions
  dim_boundary_conditions = getValue(solfile,'dim.boundaryconditions');
  dim_constants = getValue(solfile,'dim.constants');
  dim_control = getValue(solfile,'dim.control');
  dim_parameters = getValue(solfile,'dim.parameters');
  dim_path_constraints = getValue(solfile,'dim.pathconstraints');
  dim_state = getValue(solfile,'dim.state');
  dim_steps = getValue(solfile,'time.steps');
  dim_stages = 1;

  % read status, iterations, objective ans constraints
  status = getValue(solfile,'Solver status:');
  iterations = getValue(solfile,'Solver iterations:');
  objective = getValue(solfile,'Objective function:');
  constraints = getValue(solfile,'Constraints violation:');

  % read time steps and stages
  time_steps = getBlock1D(solfile,'# Time steps',dim_steps+1);
  time_stages = getBlock1D(solfile,'# Time stages',dim_steps*dim_stages);

  % read state, control, parameters
  state = getBlock2D(solfile,'# State',dim_state,dim_steps+1);
  control = getBlock2D(solfile,'# Control',dim_control,dim_steps);
  parameters = getBlock1D(solfile,'# Parameters',dim_parameters);

  % +++bc and path are after multipliers, change that
  boundarycond = getBlock1D(solfile,'# Boundary Conditions',dim_boundary_conditions);
  pathcond = getBlock2D(solfile,'# Path constraints',dim_path_constraints,dim_steps+1);

  % read boundary conditions multipliers, costate
  boundarycond_multipliers = getBlock1D(solfile,'# Multipliers associated to the boundary conditions',dim_boundary_conditions);
  pathcond_multipliers = getBlock2D(solfile,'# Multipliers associated to the path constraints',dim_path_constraints,dim_steps+1);
  costate = getBlock2D(solfile,'# Adjoint state',dim_state,dim_steps);
  
end





function [value] = getValue(solfile,label)

  line = fgets(solfile);
  while isempty(strfind(line, label))
    line = fgets(solfile);
  end
  ls = split(line);
  value = str2double(ls{end-1});

end


function [block] = getBlock1D(solfile,label,dim1)

  block = zeros(dim1,1);
  line = fgets(solfile);
  while isempty(strfind(line, label))
    line = fgets(solfile);
  end
  for k = 1:dim1
    line = fgets(solfile);
    block(k) = str2double(line);
  end

end


function [block] = getBlock2D(solfile,label,dim1,dim2)

  block = zeros(dim2,dim1);
  line = fgets(solfile);
  for i = 1:dim1
    while isempty(strfind(line, label))
      line = fgets(solfile);
    end
    for k = 1:dim2
      line = fgets(solfile);
      block(k,i) = str2double(line);
    end
  end

end
