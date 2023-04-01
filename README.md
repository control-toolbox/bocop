# Bocop3 - optimal control toolbox (direct transcription approach)

[gh-ci-img]: https://github.com/control-toolbox/bocop.jl/actions/workflows/ci_linux.yml/badge.svg?branch=main
[gh-ci-url]: https://github.com/control-toolbox/bocop.jl/actions/workflows/ci_linux.yml?query=branch%3Amain
 
## User install

### [Windows Installation Guide](./bocop/README_WINDOWS.md)

### [macOS Installation Guide](./bocop/README_MACOS.md)

### [Linux and WSL Installation Guide](./bocop/README_LINUX.md)


## Developper install

Before developing, you should try to install bocop as user, so that you have our usual tools to run bocop. Refer to the abovementioned installation guides.

In short, you should have:
- on Windows: Microsoft Build Tools, Miniconda, git (go to [the Git Website](https://git-scm.com) to get you started)
- on macOS: *maybe XCode*, Miniconda, git (it should be installed with XCode anyway)
- on Linux: Miniconda, git

Alternatively you can install git on any OS using conda with `conda install git` (but in this case, remember it will be tied to the conda environment where you install it.)

You should then clone the environment, create the conda environment for development and check if the tests are running correctly:

```bash
git clone https://gitlab.inria.fr/ct/bocop3
cd bocop3
conda env create -f pkg/env/bocop-linux.yaml ## or bocop-windows.yaml or bocop-macos.yaml
conda activate bocop-dev
python test_bocop.py 1 ## check that the tests are running
```

Then you can use:

 - **jupyter** `jupyter notebook bocop/notebook_test.ipynb`
 - **jupyter lab**
  ```
  jupyter labextension install @jupyter-widgets/jupyterlab-manager
  jupyter-lab bocop/notebook_test.ipynb
  ```
## See also
Try the code online with [Bocop wep app](http://control-toolbox.inria.fr/bocop-panel) (zero installation and full version)

![0DC1463B-9C56-4D01-85C5-198663A50047](https://user-images.githubusercontent.com/62183989/227493074-07842484-f573-4eb9-b728-c1c19b2a4eb1.jpeg)

## Documentation
Into folder doc/, run Doxygen
```
doxygen

```
which will generate the doxygen files in `html/` and `xml/` (used for sphinx import).
Then run Sphinx with Breathe extension to import the `xml/ files from doxygen
```
make html
```
which will put the `sphinx-rtd` files in `_build/html`.








