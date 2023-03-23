# Installation
This repository is a C++ port of the python scripts written [here](https://github.com/jelfring/particle-filter-tutorial). In order to build and run the code, the following packages must be installed:


* cmake (tested with cmake 3.14+)
* gnuplot (`sudo apt install gnuplot`)
* compiler that supports C++17 (tested with g++ 7.5)
* ninja (not required, but builds faster than `make`)
*
* also using fmt, eigen, and matplot++, but these are automatically fetched by CMake.

My primary dev environment was Ubuntu 20.04 running inside a docker container using VSCode as my IDE of choice. It should build fine outside of docker in any version of Ubuntu.   

To build after cloning and entering into repo top level directory:
```
mkdir build
cd build
cmake ..
ninja install
```
This will install all executables in an install folder inside the repo top level. Binaries will also be in `build/src/particle-filter-tutorial-cpp`

# Documentation

Details on the algorithms, background information in general and an documentation for all the main scripts can be found [in this paper](https://www.mdpi.com/1424-8220/21/2/438).

# Running the code

The main scripts are
* ``demo_running_example``: runs the basic particle filter
* ``demo_range_only``: runs the basic particle filter with a lower number of landmarks (illustrates the particle filter's ability to represent non-Gaussian distributions).

Whenver running the code, a robot localization problem will be simulated. For most scripts, the visualization below should appear.

![alt text](https://github.com/jelfring/particle-filter-tutorial/blob/master/images/running_example_screenshot.png?raw=true)

The picture shows a top view of a 2D simulated world. Four landmarks can be observed by the robot (blue rectangles). The landmark positions are given in the map and therefore are used to estimate the tru robot position and orientation (red circle). The particles that together represent the posterior distribution are represented by the green dots.

Besides the standard particle filter, more advanced particle filters are implemented, different resampling schemes and different resampling algorithms are available. This allows for trying many different particle filter is similar settings.

The supported resampling algorithms are:
* Multinomial resampling
* Residual resampling
* Stratified resampling
* Systematic resampling

Supported resampling schemes are:
* Every time step
* Based on approximated effective number of particles
* Based on reciprocal of maximum particl weight

More advanced particle filters that are supported:
* Adaptive particle filter
* Auxiliary particle filter
* Extended Kalman particle filter

# Reference
In case you use code in your work, please cite:

Elfring J, Torta E, van de Molengraft R. Particle Filters: A Hands-On Tutorial. Sensors. 2021; 21(2):438.