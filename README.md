# CarND - Model Predictive Control Project
---

## Overview
The goal of this project is to implement a Model Predictive Controller to drive a car as fast as possibly on the track safely in the simulator. The following animation shows how my controller works. The yellow line is the reference path and the green line is the MPC predicted trajectory.

![gif](docs/mpc.gif)

To see the complete video, checkout out [this link](https://youtu.be/8qDJUKbodao).

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

----
## Implementation
### MPC
[Todo]

### Polynomial Fitting
[Todo]

### Preprocessing
[Todo]

### Latency Handling
[Todo]

