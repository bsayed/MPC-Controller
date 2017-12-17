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
 