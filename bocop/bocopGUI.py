#!/usr/bin/env python
from matplotlib import use
import matplotlib.pyplot as plt
import numpy as np
import os

import shutil

import bocop

#import PySimpleGUI as sg; use('qt5agg') #plot does not currently work in this version...
#import PySimpleGUIWx as sg # seems buggy...
import PySimpleGUIQt as sg; use('qt5agg')

# Notes:
# - ipopt iterations dont show up anymore -_- (ok from python shell)
# - flush is not supported in print commands, use window.refresh() instead
# - apparently not easy to display bocop logo in Output element (maybe replace Output with Image at startup ?)

########################################################################
########################################################################
# Global variables
root_path = '.'
problem_path = None

########################################################################
########################################################################
# Event functions

def showAbout():
  text = 'Bocop {}\nLicense: {}\nAuthors: {}'.format(bocop.__version__, bocop.__license__, bocop.__author__)
  sg.popup(text, title='Bocop3')


def newProblem(window):
  global root_path, problem_path
  foldername = sg.popup_get_folder('Select folder for new problem',title='New problem',no_window=True)
  if foldername not in (None,''):
    bocop.newProblem(foldername)
    problem_path = foldername
    print('New problem created at: {}'.format(problem_path))
    #+++ call loadProblem instead
    window['-CurrentProblem-'].update(value='Current problem: {}'.format(problem_path))
  else:
    print('Canceled...')
     

#+++ later call this after newProblem
def loadProblem(window):
  global root_path, problem_path
  filename = sg.popup_get_file('Select problem definition',title='Select problem',initial_folder=root_path+'/examples',file_types=(('Problem files','*.def'),),no_window=True)
  if filename not in (None,''):
    
    # update problem path
    problem_path = os.path.dirname(filename)
    print('Current problem set to: {}'.format(problem_path))
    window['-CurrentProblem-'].update(value='Current problem: {}'.format(problem_path))
    # load .def file in tab2
    deffile = filename
    with open(deffile,'r') as f:
      file_text = f.read()
    window['-ProblemDefinition-'].update(value=file_text) 
    # load .cpp file in tab3
    cppfile = os.path.splitext(filename)[0] + '.cpp'
    with open(cppfile,'r') as f:
      file_text = f.read()
    window['-ProblemFunctions-'].update(value=file_text)    
  else:
    print('Canceled...')


def saveProblem(window, values):
  if problem_path is not None:
    # save .def file from tab2
    deffile = problem_path + '/problem.def'
    with open(deffile,'w') as f:
      f.write(values['-ProblemDefinition-']) 
    # save .def file from tab2
    cppfile = problem_path + '/problem.cpp'
    with open(cppfile,'w') as f:
      f.write(values['-ProblemFunctions-']) 


def plotSolution(solution_file = None):
  global root_path, problem_path
  # choose solution file if not specified
  if solution_file is None:  
    if problem_path is None:
      preload_sol = root_path+'/examples'
    else:
      preload_sol = problem_path+'/problem.sol'
    solution_file = sg.popup_get_file('Select solution',initial_folder=preload_sol,file_types=(('Solution files','*.sol'),),no_window=True)
  # load and plot solution
  if os.path.isfile(solution_file):
    solution = bocop.readSolution(solution_file)
    solution.plot(graph=1, blck=False)
  else:        
    print('Cannot find solution file: ',solution_file)


def buildProblem(window, debug):
  global problem_path
  if problem_path is None:
    loadProblem(window)
    window.refresh()
    buildProblem(window, debug)
  else:
    print('Building problem at: {}'.format(problem_path))
    window.refresh()
    bocop.build(problem_path=problem_path, verbose=1, clean=1, debug=debug, window=window)

# still some lag in ouptut here, see bocop.py
def runProblem(window, auto_plot = True):
  global problem_path
  if problem_path is None:
    print('Please load a problem first...') #+++later use popup or even launch loadProblem directly ? same with build if exec missing
  else:
    print('Running problem at: {}'.format(problem_path))
    window.refresh()
    bocop.run(problem_path=problem_path, verbose=1, clean=1, graph=0, window=window)

    # plot solution after run if needed 
    if auto_plot:
      plotSolution(solution_file = problem_path+'/problem.sol')


########################################################################
########################################################################
# bocop3 gui
def bocop3_gui():

  # main gui options
  sg.theme('Default1')
  sg.set_options(element_padding=(0, 0), font='Courier 11')

  # menu definition
  menu_def = [
              ['&File', ['&New problem', '&Load problem', '&Save problem','---', '&Plot solution','---', '&Exit']],
              ['&Build and Run', ['&Build', 'Build (&Debug)', '&Run']],
              ['&Info...', ['&About']],
             ]

  # icons
  icon_new = sg.Button('',image_filename='./icons/new.png', image_size=(32, 32), key='New_icon')
  icon_load = sg.Button('',image_filename='./icons/load.png', image_size=(32, 32), key='Load_icon')
  icon_save = sg.Button('',image_filename='./icons/save.png', image_size=(32, 32), key='Save_icon')
  icon_build = sg.Button('',image_filename='./icons/build.png', image_size=(32, 32), key='Build_icon')
  icon_run = sg.Button('',image_filename='./icons/run.png', image_size=(32, 32), key='Run_icon')
  icon_plot = sg.Button('',image_filename='./icons/plot.png', image_size=(32, 32), key='Plot_icon')
  icon_kill =sg.Button('',image_filename='./icons/kill.png', image_size=(32, 32), key='Kill_icon') # +++ to kill build/run

  # checkbox
  plot_after_run_cb = sg.Checkbox('Plot after run', default=True)

  # GUI layout
  tab1_layout = [[sg.Output(size=(80,20), key='-MAIN-')]]
  tab1 = sg.Tab('Main window', tab1_layout, background_color='white')
  tab2_layout = [[sg.Multiline('Please select a problem', key='-ProblemDefinition-', size=(80,20))]]
  tab2 = sg.Tab('Problem definition', tab2_layout, background_color='white')
  tab3_layout = [[sg.Multiline('Please select a problem', key='-ProblemFunctions-', size=(80,20))]]
  tab3 = sg.Tab('Problem functions', tab3_layout, background_color='white')  

  layout = [
            [sg.Menu(menu_def, tearoff=False, pad=(20,1))], #option font in later versions, upgrade
            [icon_new, icon_load, icon_save, icon_build, icon_run, icon_plot, plot_after_run_cb],
            [sg.TabGroup([[tab1, tab2, tab3]])],
            [sg.Text('Current problem: {}'.format(problem_path), key='-CurrentProblem-', font='Courier 11 bold')],
           ]

  main_window = sg.Window('Bocop 3', layout, resizable=True, return_keyboard_events=True)
  #main_window['-MAIN-'].expand(expand_x=True, expand_y=True) does not work

  # Loop on events
  while True:

    event, values = main_window.read()
    if event in (None,'Exit'):
      return

    # callbacks for menu choices
    if event == 'About...':
      showAbout()

    # load problem / solution
    elif event in ('New problem', 'New_icon'):
      newProblem(main_window)    
    elif event in ('Load problem', 'Load_icon'):
      loadProblem(main_window)
    elif event in ('Save problem', 'Save_icon'):
      saveProblem(main_window, values)      
    elif event in ('Plot solution', 'Plot_icon'):
      plotSolution()
    
    # build problem
    elif event in ('Build', 'Build_icon'):
      buildProblem(main_window, debug=0)
    elif event == 'Build (Debug)':
      buildProblem(main_window, debug=1)
      
    # run problem
    elif event in ('Run', 'Run_icon'):
      runProblem(main_window, auto_plot=plot_after_run_cb.get())


# launch gui ^^
if __name__ == "__main__":
    bocop3_gui()



# TODO

# - add icons for load, save, build, run, plot: beautify a bit (spacing). Icons dont show properly on mac -_-
# - use different colors for text output (errors in red, etc)
# - make gui resizable (output element mostly)   window['_BODY_'].expand(expand_x=True, expand_y=True) ?
# - add button to kill current build/run
# - robustify runProblem: loadProblem if none, buildProblem if no exec (check return code for missing exec ?)
# - buildProblem: infinite loop in loadProblem if Cancel
# - bloody freaking macos puts the menu on top of the screen, plus popups all FAIL ('Canceled')
# - refresh .def and .cpp files at click (currently does not show outside modifications)
# - switch to tab1 when doing build/run
