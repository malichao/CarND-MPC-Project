# CarND - Model Predictive Control Project
---

## Overview
[Todo]

## Dependencies
I created a docker image that includes all the dependencies. Apart from those, I managed to get Qt creator to work in the container so that I could debug my program. To build the image: 

```
cd docker
./build.sh
```
The script will create an image called *carnd-term2*. To run the container:

`./run_term2_docker.sh`

It will mount the **current** directory into the container as **/src**. That means you could keep all the source code outside of the container.

If you don't want to install this image, then follow [udacity's project repo](https://github.com/udacity/CarND-MPC-Project) to set up the environment.


## Build & Run

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc [path_to_config]`. Config file is optional, by default it loads "config/test.cfg".


## Implementation
[Todo]

### MPC
[Todo]

### Polynomial Fitting
[Todo]

### Preprocessing
[Todo]

### Latency Handling
[Todo]

