# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

In this project you'll revisit the lake race track from the Behavioral Cloning Project. This time, however, you'll implement a PID controller in C++ to maneuver the vehicle around the track!

The simulator will provide you the cross track error (CTE) and the velocity (mph) in order to compute the appropriate steering angle.

One more thing. The speed limit has been increased from 30 mph to 100 mph. Get ready to channel your inner Vin Diesel and try to drive SAFELY as fast as possible! NOTE: you don't have to meet a minimum speed to pass.
---

### Reflection on Hyper-Parameter Tuning:

### Impact of Parameters
The parameters whose values are needed to be determined in order to calculate the steering angle are Kp, Ki and Kd.

1) Impact of Kp
Utilizing only the coefficient Kp by setting the other coefficients to 0, is expected to provide considerably large oscialltory behaviour of the car around the simulator. This is indeed the case as can be seen in the video below.

[![P-Controller](http://img.youtube.com/vi/5Oc0wyf6QdQ/0.jpg)](http://www.youtube.com/watch?v=5Oc0wyf6QdQ "P-Controller")

Here the car starts oscillating and eventually rolls over the edges of the road.

2) Impact of Kp and Kd
As shown above, the utilization of coefficient Kp is not sufficient to ensure that the car drives around the track successful. The derivative coefficient Kd assists in this case as its impact should have been to reduce the oscialltory behaviour of the car when driving around the track. This is validated by setting Ki to 0 and setting the other two coefficients Kp and Kd to non-zero values.

[![PD-Controller](http://img.youtube.com/vi/t1bP40pRMC4/0.jpg)](http://www.youtube.com/watch?v=t1bP40pRMC4 "PD-Controller")

Here the car's oscillatory behaviour is reduced to a large extent and the car can drive around the simulator track.

3) Impact of Kp and Kd and Ki
With the P-D Controller, the car can successfully drive around the track with the chosen values but still grazes the outside edges of the lane which can be considered as unsafe drving.
In order to further reduce the osciallatory behaviour of the car, Ki was set to a very small value. The impact was assumed to be minor with respect to the overall driving behaviour of the car. As expected there is not much impact except the turns where the car performs better than the PD Controller and maintains a safer distance from the lane edges.

[![PID-Controller](http://img.youtube.com/vi/JKg5bSQkxGo/0.jpg)](http://www.youtube.com/watch?v=JKg5bSQkxGo "PID-Controller")


### Method used for Hyper-Parameter Tuning

In order to derive the set of paramter values, two steps were followed:

a) Manual tuning to derive paramter settings which are good enough for the car to complete driving around the track.

b) Automated paramter tuning using the Manually tuned paramters settings using Twiddling.

Steps followed for Manual tuning of parameters:
-----------------------------------------------
a) First I tried setting various values of Kp starting from 0.005.

b) The value of paramter Kd was then set to a value of 1.
With the above two set, I observed that the car was able to cover some portion of the track wich validated that that the impact of Kp and Kd was as expected. I had initally set the value of Ki to 0, because in the simulator, I assumed that the wheel alignment was almost correct and there was no bias with respect to it.

c) Further I introduced a very small value of Kp as 0.001 and observed that there was barely any impact.

d) The next step was obtain a balance between Kp and Kd i.e. increase in Kp implied Kd had to be decreased a bit and any decrease in Kp or Kd only implied that a slight increase in Ki would be able to compensate for it.

With thea above iterative approach, I finalized on a Kp value of 0.10, Ki value of 0.002 and Kd value of 2.
The next-step was an automated tuning mechanism based on Twiddle with the Kp, Ki and Kd start values indicated above.

Steps followed for Automated Parameter tuning using Twiddle:
--------------------------------------------------------------
The Hyper-parameter values obtained from Manual tuning were utilized as a starting value of the Automated parameter tuning algoruthm.

a) An instantaneous CTE threshold of 4 was identified as a point to reset the simulator and try attempt selecting the next iterative choice of hyper-parameters as per Twiddle algorithm. This was done as any absolute value of CTE that exceeded the CTE threshold, implied that the car would touch or topple beyond the lane edges. This was determined empirically. If the simulator is reset due to the CTE threshold being crossed, it will be assumed that the average state of the currently accrued error is worse than the best recorded error and the next choice of hyper-parameters should be chosen from Twiddle algorithm under this assumption.

b) Each choice of hyper-parameters chosen as per Twiddle algorithm would be executed for 1150 time-steps i.e. almost one lap of the simulator has to be covered for comparing the average accumululated error to the Best error.

c) The simulator would be reset with the next available choice of Hyper-paramters when the car completes one lap around the track with the current choice of hyper-parameters i.e. the the simulator should start reporting CTE with the next available choice of Hyper-parameters from the start of a new lap around the track.

d) The Twiddle algorithm stated in the lectures suggested a recording of the total error once a few time-steps have passed i.e. to allow for the case of initial oscillatory nature of the car with these parameters. This has not being followed in this implementation as bigger oscillations at the starting phase of the lap can cause the car to go beyond the car edges.

e) Depending upon the starting point of the Kp, Kd and Ki values and the ranges observed during manual tuning, the delta values(referenced as dp_params in code) has been set to reasonable values that are good enough to cover the probable maxima around the initially tuned parameter values. Setting them all to consistently higher values caused the Automated tuning process to go on for a large amount of time.

f) The hyper-parameter values that correspond to the best accrued CTE error through the duration of the lap have been retained(best_params in code) and will be the end result of the Twiddle algorithm. This has been done as it was observed that the final setting of Hyper-parameters when the Twiddle algorithm exits was too dependent on the sum of the deltas and need not necessarily be the best maxima and can possibly end up being one of the local maximas next to the global maxima obtained.

The Twiddle algorithm with the above assumptions was executed twice with different Hyper-param start values and CTE Thresholds as below:

1) The first result was the set of Hyper-paramter values of Kp=0.30, Ki=0.0002, Kd=3 when started with the results of the Manual Tuning process and CTE threshold of 4.

2) The second iteration was started with the obtained values of the first iteration of Twiddle algorithm and a lower CTE threshold of 2.5.

The second result was the final set of Hyper-parameter values of Kp=0.389084, Ki=0.00152451, Kd=3.41323 and these values were chosen for the final implemenation.

Note:
------
It was observed that even after issuing the reset message from the Client to the Simulator server, there was a significant time-lag between the command being issued and the Simulator actually being reset. That is, some CTE readings could still be passed onto the Client even after issue of Reset to Simulator. This necessiated avoiding any toggling of Hyper-parameters values(i.e. execution of Tiwddle) after issue of Reset to Simulator, i.e. any CTE values obtained after issue of Reset will be ignored in the implementation until the simulator is actually reset.
