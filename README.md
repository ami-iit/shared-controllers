# Shared Controllers

This repository provides the following software:
- [`lib`](lib): contains `wholebodycontrollib`, a python library for controlling humanoid robots.
- [`scripts`](scripts): contains python applications to run whole-body-controllers on real or simulated robots.


## Dependencies
- [`python3`](https://wiki.python.org/moin/BeginnersGuide) (< 3.11)
- [`matplotlib`](https://github.com/matplotlib/matplotlib)
- [`qpsolvers`](https://github.com/qpsolvers/qpsolvers)
- [`yarp`](https://github.com/robotology/yarp)
- [`idyntree`](https://github.com/robotology/idyntree)

Optional
- [`bipedal-locomotion-framework`](https://github.com/ami-iit/bipedal-locomotion-framework)
- [`icub-models`](https://github.com/robotology/icub-models)
- [`gazebo-yarp-plugins`](https://github.com/robotology/gazebo-yarp-plugins)

## Installation

### Dependencies
All the required dependencies can be installed via `conda`.

Create the conda environment:
```
conda create -n sc_env python=3.10
conda activate sc_env
```

Install dependencies:
```
conda install -c conda-forge pip matplotlib qpsolvers resolve-robotics-uri-py
conda install -c conda-forge yarp idyntree
```
**Optional**
In order to run the controllers and the simulations, additional packages are required:
```
conda install -c conda-forge gazebo-yarp-plugins icub-models bipedal-locomotion-framework
```

### Library
In order to install the library, activate the environment in which the dependencies are installed:
```
conda activate sc_env
```

Clone the repository and install the package:
```
git clone https://github.com/ami-iit/shared-controllers.git
cd shared-controllers
pip install .
```
