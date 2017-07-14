# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Rubric Points Discussion

### Student describes their model in detail. This includes the state, actuators and update equations.

A basic kinematic model is used in the project to approximately estimate/update the position, orientation, velocity, cross track error and error in orientation of the car. These parameters form the state of the model. The actuators modeled are throttle and steering angle. Negative throttle is used as a proxy for braking. The advantage of using such a model is simplicity - both in terms of implementation and runtime efficiency. However, such a model does not capture all the possible forces acting upon a car in precise detail. Therefore it tends to be a little less accurate than complex dynamic models. At slow speeds though this would not matter too much.

### Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

The key idea behind the choice of N and dt is to figure out the right tradeoff between prediction horizon, accuracy and computational efficiency. Given that we're using an approximate model, I figured trying to predict too far into the future would cause inaccuracies. So I settled on predicting about 1 second into the future. Then I tried tweaking N and dt to achieve this as follows: (50, 0.02), (20, 0.05), (10, 0.1). The first one clearly took a while to run and did not seem to fit the real-time requirements. The second and third choices worked fine on my laptop. I chose the 3rd choice because it is computationally more efficient. I did not try dt value larger than 0.1 because I felt that it would be too much of discretization of our continuously valued state variables.

### A polynomial is fitted to waypoints. If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

I did the following pre-processing steps before invoking MPC:

* Convert velocity from MPH to M/S for the sake of the equations
* Transform car's x,y to be the origin (0,0). Then rotate all waypoints of "ideal" path given by simulator (in map coordinates) relative to this new origin. This has the effect of making psi, the car's orientation w.r.t lane center, zero. It also makes computing cross track error and orientation error very simple. The approximate CTE becomes the y-distance between the origin (or car's position) and the ideal path, and the approximate orientation error becomes the negative arctan of the first polynomial coefficient when the derivative of the polynomial is evaluated at x=0.
* Also, the angles returned from the simulator were reversed while using in kinematic equations to account for differences in the simulator's directional interpretation and that of the car's.

### The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

I basically dealt with the latency of 100 milliseconds by doing the following things:

* While returning the delta (steer) and the acceleration (throttle) values from  the MPC::solve() function, instead of returning the instantaneous current values, I return the value at 0 + x where x is ceil(latency/dt) which in my case was 1. So essentially, I return what the model thinks the actuation should be 1dt or 100ms into the future as per my parameters.
* To deal with any issues due to this approximation into the future, I also added an extra component to the cost function where I penalize simultaneous high values for steering and throttle - high speed and high steering input is bad. This intuitively makes sense. The cost component that captures that is: (steer * throttle)^2. This proved especially useful for negotiating tricky turns.

