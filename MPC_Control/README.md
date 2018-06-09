Model Predictive Controller - Udacity Self-Driving Car Engineer Nanodegree Program
-----------------------------------------------------------------------------------

In this project you'll implement Model Predictive Control to drive the car around the track. This time however you're not given the cross track error, you'll have to calculate that yourself! Additionally, there's a 100 millisecond latency between actuations commands on top of the connection latency.


Project Repository and Build Instructions: https://github.com/udacity/CarND-MPC-Project
The simulator utilized in this Project is found at: https://github.com/udacity/self-driving-car-sim/releases

## Project Details:

### The Model Description

The State maintained as part of the Model Predictive Controller are the X coordinate, Y coordinate, Heading of the car(Orientation), Magnitude of Velocity, Cross Track Error, Error in Heading with all of them with respect to the Vehicles local coordinate system.

The Model aims to find the actuator control inputs i.e. the steering angle and the throttle for the next time-step based on the input Reference Trajectory that has been fed from the simulator. In order to find the next time-step, an Optimizer Ipopt(https://projects.coin-or.org/Ipopt) is utilized to find the actuator control inputs for 'N' time-steps and then utilize the values obtained for the next time-step only  in order to move the vehicle forward in time.

The determination of N time-steps in order to determine the control inputs necessary for the next time-step only provides a time-horizon where the Objective Cost function is trying to match the variable inputs contributing towards the cost function as closely as possible to the Reference values.

The Ipopt library usage requires the setting of the objective function, the constraints and their associated lower and upper bounds as well as a lower and upper bound on the variables involved.

The model features update of the state variables of the Controller by the equations for variable state other than the current state:
```
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) - v0 * delta0 / Lf * dt);
```
 where the variables indexed with 1 indicate the next state and the variables indexed with 0 indicate the current state being processed.
 
Each of these equations are originally constraints whose lower and upper bounds for each of the states other than the current state are set to 0 as follows so that the model equations are satisifed over all following time-steps:

```
 for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
```

The constraints for the current state are set to the current state variable values as well as their upper and lower bounds are set to the value of the current state, since they are not changeable.

The cost function, features the following contributors to cost:
 
 1) Minimization of the total Cross-Track Error across all time-steps relative to the Ref Cross-Track error of 0, i.e. trying to drive it towars 0.
 
 2) Minimization of the total Error in Heading across all time-steps relative to the Ref Heading error of 0, i.e. trying to drive it towards 0.
 
 3) Trying to maintain a reference velocity of 100 across all time-steps.
 
 4) Minimization the use of actuators throughout all time-steps.
 
 5) Mimization of the changes for actuator inputs across time-steps so that the Car does not accelerate too fast or wobble around the center of the lane due to rapidly changing Actuator control inputs of steering angle and Throttle.
 
Further each of the above mentioned contributors of cost have been weighted relative to each other so that some of the above contributors are considered more relevant for the cost function minimization relative to the others.

### Timestep Length and Elapsed Duration (N & dt)

The optimizer utilized in this project calculates the optimal set of control inputs(Steering Angle and Throttle) that are suitable for the defined time-horizon. The total time-horizon is defined as the product of the two chosen parameter values: N and dt where N stands for the total number of time-steps in the time-horizon and dt stands for the time-interval inbetween two consecutive time-steps.

Since the Track driven by the car in the simulator features several left and right turns and some very sharp turns as well, defining a very long time-horizon would not work due to the rapidly changing trajectory especially at the high Reference velocity of 100 mph that has been utilized.As a result of the this, I have only attempted to find the best balance between N and dt that gives a time-horizon in the range of 1 to 3 seconds.

I initially tried with the below total Time-horizons are their results were:

N= 300 and dt = 0.01:
Result is that car drives over the lane edges as soon as it starts.

N= 200 and dt = 0.01:
Result is that car drives over the lane edges as soon as it starts.

N= 100 and dt = 0.01:
Same result as above.

Further I went ahead with the constraint that a time-horizon lesser than 1 second would be too short and computationally expensive to get a good enough result on my computer.

Next I tried reducing the N value and increasing the dt value and the results were:

N=30 and dt = 0.1: 
Results wree much better and the car travels a significantly large distance with a wobbling predicted trajectory but fails to make the sharp left turn after the bridge.

N=20 and dt =0.01: 
The car is now able to make all the turns successfully but there is still some oscialltory behaviour of the predicted trajectory around the lane turns. Also the car does not maintain a safe distance around lane edges for turns which can be considered unsafe driving.

N=10 and dt =0.1:
The car is able to make all the turns successfully and there is significantly less oscialltory behaviour around the lane turns as well as the Car maintains a safe distance around the lane edges for turns.

With the above set of trials I fixated on the values of N=10 and dt=0.1 i.e. a time-horizon of 1 second that worked best on the simulation run for my computer. Further I tuned the weights of the factors contributing towards the cost function in order to achieve a smoother and safer driving trajectory.

The realization from the tuning of these parameters was that, a shorter time-horizon worked best for a track with sharp turns. Also a bigger gap between time-steps worked better than a shorter gap between time-steps since it probably allows the Optimizer to determine a smoother Trajectory that resembles the Reference trajectory best.


### Polynomial Fitting and MPC Preprocessing

The determination of the Reference Trajectory coefficients was done as follows:

a) Process the Reference Trajectory waypoints provided by the Simulator in the global coordinate system and convert them in the vehicle's local coordinate system.

b)  Attempt to find a 3rd order polynomial fit to the waypoints in the vehicle coordinate system. A 3rd polynomial made sense since the turns of the track were evenly spaced out(there was significant distance between them) and it looked sufficient. A track with back to back turns would possibly require a higher order polynomial fit for good results.

c) Utilize the polyeval helper function to determine the 3rd order polynomial coefficients that are a good fit for the Reference trajectory.

Further the state variables that were directly provided by the Simulator were the X coordinate, Y coordinate, Veocity Magnitude, Heading, Steering angle and current Throttle value.

Since the entire MPC model was to be operated with respect to the Vehicle coordinate system, the X coordinate, Ycoordinate and Heading would all be 0 since the vehicle would be at the origin of its own coordinate system at the initial state fed by the simulator.

The CTE and the Error in Heading were calculated with the above siwtch to the vehicle coordinate system and the Velocity Maginitude was utilized as was provided by the Simulator.

For a system without latency, these values of the state obtained with respect to the Vehicle coordinate system would be the initial state to be fed into the MPC Optimizer. 

However in this project, we are taking latency of the control input command being issued to the vehicle and its propagation delay to the system into account. This is achieved by putting the Main function thread to sleep for 100 ms, i.e. a latency of 100 ms has been modeled in this project.

In order to account for latency, the approach followed has been desribed below.

### Model Predictive Control with Latency

In order to deal with latency of 100 ms, I took the initial state variables obtained with respect to the Vehicle Coordinate system above and extrapolated them to a time of 100 ms into the future with the below equations:

```
   //Future values of state with latency
   double fut_px = 0 + v*std::cos(0)*latency;
   double fut_py = 0 + v*std::sin(0)*latency;
   double fut_psi = 0 - (v/Lf)*delta*latency;
   double fut_v  = v + acceleration*latency;
   double fut_cte = cte + v*sin(epsi)*latency;
   double fut_epsi = epsi - (v/Lf)*delta*latency;
```

These time-forward projected values of the initial state with respect to the Vehicle Coordinate system were fed into the MPC Controller to find the optimized set of control inputs for the next time-step.

### Results

The results of the MPC Controller are as follows:

[![MPC Controller](http://img.youtube.com/vi/5QymABWDYpo/0.jpg)](http://www.youtube.com/watch?v=5QymABWDYpo)

The yellow line in the video shows the Reference Trajectory and the Green line shows the Predicted Trajectory.

### References:

Some of the approaches taken for this project have been inspired by the useful tips and tricks provided in the Q&A session: https://www.youtube.com/watch?v=bOQuhpz3YfU&feature=youtu.be

