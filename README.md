# CarND-Controls-MPC

This project implements a Model Predictive Control to drive a car around a track in a simulator. 
The cross track error is calculated by the algorithm. 
Additionally, there's a 100 millisecond latency between actuation commands on top of the connection latency.

---
The following is a GIF generated from driving the car around the track, displaying the MPC trajectory path in green, and the polynomial fitted reference path in yellow.

[gifImage]: ./MPC.mp4.gif

![alt text] [gifImage]


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
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * If challenges to installation are encountered (install script fails).  Please review this thread for tips on installing Ipopt.
  * Mac: `brew install ipopt`
       +  Some Mac users have experienced the following error:
       ```
       Listening to port 4567
       Connected!!!
       mpc(4561,0x7ffff1eed3c0) malloc: *** error for object 0x7f911e007600: incorrect checksum for freed object
       - object was probably modified after being freed.
       *** set a breakpoint in malloc_error_break to debug
       ```
       This error has been resolved by updrading ipopt with
       ```brew upgrade ipopt --with-openblas```
       per this [forum post](https://discussions.udacity.com/t/incorrect-checksum-for-freed-object/313433/19).
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/).
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 
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


## Code Style

[Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).


## Model
### State
The MPC controller depend on the following set of variables that represent the state of 
the system.
- `x` (double) The x position of the vehicle.
- `y` (double) The y position of the vehicle.
- `psi` (double) The vehicle orientation.
- `v` (double) The vehicle velocity.
- `cte` (double) The cross track error, calculated as the difference of the fitting
the current x position and the y position we get from the simulator. `cte = polyeval(coeffs,x) - y`.
- `epsi` (double) The orientation error, calculated as the derivative f'(x) where f(x) is equal to 
`coeffs[0] + coeffs[1] * x` which results in `coeffs[1]`. The orientation error final value is 
calculated as `-atan(coeff[1])`, the negative sign is added because of the way the simulator
is considering positive angles in the car coordinate system as negative value 
(e.g. when steering to the left).

### Actuators
- `delta` The heading angle. It is limited to [-25 degs, 25 degs] by the simulator.
- `a` The acceleration of the vehicle which can range [-1, 1].

### Update Equations
The following update equations resembles the ones described in the lesson, `dt` is the
time elapsed between timesteps. The `t` var ranges over the number `N` of timesteps that we have.

`for t=1 to N`
- `x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt`
- `y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt`
- `psi_[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt`
- `v_[t] = v[t-1] + a[t-1] * dt`
- `cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt`
- `epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt`

## Timestep Length and Elapsed Duration (N & dt)
The `N` was chosen to be 12 which was a tradeoff between how many points to predict
and the computational complexity. A bigger `N` would not make a difference as we usually
need the first few predicted points to control the vehicle. I have tried 24 and 50 with
no effect on accuracy of the model.

The `dt` was chosen to be 0.05 sec or 50 milliseconds so that it is not too frequent
nor too relaxed. I have tried 100 milliseconds which also worked but higher than that
the update to the car actuators was not good enough to keep the car on track, e.g. 500 
milliseconds will take the car off track.

## Preprocessing 
The waypoints are transformed to the vehicle coordinate system before processing. I used the following
formulas to perform the translation and rotation with respect to the vehicle global position `px` and `py`:
- `(cos(psi) * (ptsx[i] - px)) + (sin(psi) * (ptsy[i] - py))`
- `(-sin(psi) * (ptsx[i] - px)) + (cos(psi) * (ptsy[i] - py))`

The `state` of the vehicle is always assumed it starts at `(x,y)=(0,0)` with orientation
equals to `psi=0`.

## Latency
The code simulates a latency of 100 milliseconds. The developed model deals with this 
latency by dividing the latency by the `dt` and using that as an index to return a zero
based index of the actuators values returned by the solver. The idea is that if we know,
for example, that the vehicle has a 100 milliseconds latency we should send the actuation
commands that are 100 msec ahead in time. In the case of the 100 msec latency and `dt`
equals to 50 this gives us 100/50 = 2, so we return the third actuation command from the
angle and throttle arrays returned by the solver.
 
