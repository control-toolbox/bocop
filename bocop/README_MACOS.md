# macOS Installation guide

## On macOS: you may need to install XCode

As this has been untested on macOS, you may (or may not) need to install XCode to have macOS build tools. You can try without XCode, and if compiling fails, try to install it.

## Install an anaconda/miniconda distribution

Anaconda is a Python distribution and a package manager for all operating systems. The Control Toolbox suite is shipped on Anaconda to simplify installation for all users.

Install:
- [Miniconda3 (~60MB)](https://repo.anaconda.com/miniconda/Miniconda3-latest-MacOSX-x86_64.sh) (from [Miniconda website](https://docs.conda.io/en/latest/miniconda.html)).

To install it, open a terminal, go to your location with `cd`, add executable rights to the file with `chmod +x Miniconda3-latest-Linux-x86_64.sh` and run the file with `./Miniconda3-latest-Linux-x86_64.sh`.

Make sure you enable the Miniconda3 initialization when asked.

```bash
Do you wish the installer to initialize Miniconda3
by running conda init? [yes|no]
[no] >>> yes
```

Alternatively you may find on Miniconda website a PKG version to install Miniconda with a graphical user interface.

<!--
Alternatively you could install the complete [Anaconda3 distribution (~500MB)](https://repo.anaconda.com/archive/Anaconda3-2020.07-Windows-x86_64.exe), but it is not recommended since it doesn't add any useful features for our use and it's particularly heavy.
-->

## Install bocop

Opening a new shell, you should now have a conda environment activated (look for ` (base) ` written at the left of your command prompt)). You should now create a conda environment for bocop, switch to that environment, and check that the examples compile and pass successfully.

```
> conda create -n bocop
> conda activate bocop
> conda install -c dtk-forge -c conda-forge -c control-toolbox bocop
```

You can check that bocop tests pass from a python shell:

```
> python
>>> import bocop
>>> bocop.test()
```

## Use bocop in Jupyterlab Notebook

Before using Jupyter-lab you must install some jupyterlab extension (not needed for jupyter).

```
jupyter labextension install @jupyter-widgets/jupyterlab-manager
```

Then from a command shell with your environment **bocop** activated, you can run
```
jupyter-lab ${CONDA_PREFIX}\lib\site-packages\bocop\notebook_test.ipynb
```
And run the cells.
