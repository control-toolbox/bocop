# Installation guide

The preferred way to install bocop3 on windows is through WSL (Windows Subsystem for Linux).
WSL can be installed on Windows 10 and 11 using the Microsoft Store application. 
Once a WSL console is available, the installation follows the same procedure as for [Linux](./bocop/README_LINUX.md)

## On Windows: install Microsoft Build Tools

Microsoft Build Tools is the complete tool suite to compile programs on Windows. It is mandatory for bocop on WIndows since it has to compile a binary library every time you need to compute the solution to a new problem.

Install:
- [Microsoft Visual Studio Build Tools](https://visualstudio.microsoft.com/fr/thank-you-downloading-visual-studio/?sku=BuildTools&rel=16).

When running the installer, select **C++ Build Tools**. This should take something between 4 to 6GB. You may need to reboot your computer afterwards.

<!--
Alternatively you could install Microsoft Visual Studio with the **C++ BUild Tools**, but if you don't use Visual Studio, the build tools alone are sufficient.
-->

## Install an anaconda/miniconda distribution

Anaconda is a Python distribution and a package manager for all operating systems. The Control Toolbox suite is shipped on Anaconda to simplify installation for all users.

Install:
- [Miniconda3 (~60MB)](https://repo.anaconda.com/miniconda/Miniconda3-latest-Windows-x86_64.exe) (from [Miniconda website](https://docs.conda.io/en/latest/miniconda.html)). Install with default options. You can untick "Register Anaconda in the shell", to keep your OS nice and tidy.

<!--
Alternatively you could install the complete [Anaconda3 distribution (~500MB)](https://repo.anaconda.com/archive/Anaconda3-2020.07-Windows-x86_64.exe), but it is not recommended since it doesn't add any useful features for our use and it's particularly heavy.
-->

## Install bocop

You should now have a shortcut in your start menu called **Anaconda Prompt (miniconda3)**. You should now create a conda environment for bocop, switch to that environment, and check that the examples compile and pass successfully.

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

Then from a command prompt with your environment **bocop** activated, you can run
```
jupyter-lab %CONDA_PREFIX%\lib\site-packages\bocop\notebook_test.ipynb
```
And run the cells. They should compile and display graphs.


# Windows Log

This is a quick summary of what has been made to run bocop on Windows.

- IPOPT is version 3.12.13 on Windows and has been compiled and uploaded by Nicolas Niclausse on the conda channel **dtk-forge**.
- MUMPS-SEQ is version 5.1.2 on Windows, must be called `dmumps.dll`, and has been compiled and uploaded by Nicolas Niclausse originally on the conda channel **dtk**. It has been re-uplaoded on the channel **dtk-forge** to ensure consistency and only referencing one channel. The latest version of MUMPS-SEQ (5.2.1) does not converge successfully on Windows.
- The toolchain to compile problems is **Microsoft Build Tools**. You can install the Microsoft Build Tools with C++ Build Toolsalone or install the complete Visual Studio Community Edition with the C++ Build Tools.
- You must use the shortcut **Anaconda Prompt (miniconda3)** on Windows to run a command prompt with Conda available.
- **Compilation in Debug mode is not available on Windows** because Anaconda.org does not ship Python libs with debug symbols. If you try to enable Debug mode on Windows **it will switch back automatically to Release mode** and it will display a short warning message.
