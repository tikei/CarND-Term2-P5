# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---


## Project Description

Autonomous steering and throttle control using a Model Predictive Controller in C++.
This project uses the Udacity Self-Driving car simulator. 

#### Demo: Autonomous driving using MPC for steering and throttle: (click on link to see the full video)

[Demo video](https://youtu.be/yPhnZ-SN2zM)


## MPC Controller 

The vehicle model used in this project is the Global Kinematic Model. The state of the vehicle includes its coordinates, velocity,  orientation. This is a simplification of the Dynamic Vehicles Models, which also include tire forces, longitudinal and lateral forces as well as air resistance, drag etc.
The equations of the Global Kinematic Model to update the vehicle state are:

*x_t+1 = x_t + velocity_t * cos(psi_t) * dt*
*y_t+1 = y_t + velocity_t * sin(psi_t) * dt*
*psi_t+1 = psi_t + velocity_t / Lf * delta * dt*
*velocity_t+1 = velocity_t + a * dt*

where:

* **psi_t** : the angle of orientation at time t
* **dt** :  the elapsed time between control actuations
* **delta** : the steering control input
* **a** : the acceleration input (throttle / brake) 
* **Lf** :the distance between the front of the vehicle and its center of gravity. The larger the vehicle, the slower the turn rate. 

**Errors**
The Global Kinematic Model also considers the follwing two errors:

* The cross-track-error **CTE** and
* The orientation error relative to the reference line orientation. 
The update equations for these errors are:

* *cte_t+1 ​= cte_t​ + v_t​ * sin(psi_error_t​) * dt* (1)

where cte_t can be expressed as the difference between the line the vehicle has to follow and the current vehicle position y.
Assuming the reference line is a first order polynomial f, f(x_t) is the reference line and cte_t = y_t - f(x_t)
Therefore, substituting in (1):

* *cte_t = y_t − f(x_t) + v_t​ * sin(psi_error_t​)* * dt

The orientation error is calculated as follows:

*psi_error_t+1 = psi_error_t + v_t / Lf  * delta * dt*

where, 

*psi_error_t* is the desired orientation subtracted from the current orientation of the vehicle *psi_t*:

*psi_error_t = psi_t - psi_des_t*

The desired angle at time t **psi_des_t** is calculated as the tangential angle of the polynomial f evaluated at x_t, i.e.:
*psi_des_t = arctan( f_prime(x_t) )*, where *f_prime(x_t)* is the derivative value of the polynomial.

The Global Kinematic Model state vector is then: ( *x_t+1 , y_t+1, psi_t+1, velocity_t+1, cte_t+1, psi_error_t+1* ) 

In addition, to this state vector, the actuators (steering and throttle) are appended.

The goal of the MPC is to minimise the above state vector errors, by selecting appropriate steering and throttle values. These values are then used to steer the vehicle. 


### Parameters

The MPC algorithm uses the following parameters:
**Lf**: a value of 2.67 meters was obtained by measuring the radius formed by running the vehicle in the simulator around in a circle with a constant steering angle and velocity on a flat terrain.

**N**: the number of time steps in the horizon, considered by the MPC controller.

**dt**: how much time elapses between actuations. 

A value of 10 was selected for **N** and a value of 0.1 for **dt**. This provided good "granularity" of the model, where actuations are performed every 100 milliseconds, and the total timeframe considered is 1 second (10 * 0.1). Experimenting initially with a value of 25 for N was not successful as the horizon considered was too long, and the car was not able to stay on the track. This was especially problematic once the reference speed of the vehicle was increased to 80mph.

The cost(objective) function that the MPC algorithm seeks to minimize includes the cross-track error, the orientation error, a penalty for a velocity lower than the reference velocity. In addition, in order to smooth the actuator inputs for a smooth driving control, a penalty for large magnitude of actuator inputs as well as a penalty for large changes in control inputs between two actuations was included.

The cost function used the following **weights**, which were obtained empirically, by observing the vehicle behaviour.

* a value of 200 was used as weight for the orientation error in the cost function, giving priority of orientation error minimization over cross-track error minimization. This helped the car to avoid wobbling around the reference line.
* a weight of 50 was used to penalize for high absolute values in steering actuations.
* a weight of 1000 was used to penalize for large changes in steering input from one time period to the next. 

**ref_v** : reference speed for the vehicle. Used to penalize the vehicle for slowing down or stopping. Initially a speed of 40 mph was used. Once all the above parameters were optimized, the speed was increased to 60 and then to 80.

### Polynomial fitting

A third degree polynomial was fitted to the waypoints streamed from the simulator.These waypoints are represented in global coordinates and had to be transformed in the vehicle coordinate system. This was done by affine transformation (translation and rotation) of the *x* and *y* simulator coordinates to vehicle coordinates. 

### State vector pre-processing and latency

In addition, before the state vector was passed for optimization to the MPC algorithm, the values of the state vector were updated to account for the latency between steering / throttle commands and their actual execution by the controllers.
The above equations of the Global Kinematic Model were used to update the current state values taking into account the latency in the system. For example, the predicted cte_t value, taking into account latency, was calculated as follows:

*predicted_cte_t = cte_t + (velocity_t * sin(psi_error_t) * latency*


### Additional Throttle control optimization
As discussed, the Global Kinematic Model does not take into account various forces (tire, longitudinal, lateral) To add a more realistic behaviour, the throttle was scaled by the slope of the fitted curve evaluated 30 meters ahead of the car. By calculating the derivative (slope) of the curve at this point, the car can anticipate the radius of the next turn and slow down accordingly. Otherwise, in a realistic scenario, the car would not be able to pass all turns driving at a constant velocity of 80 mph!
 


## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.


## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
