# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

The goal of this project is to implement a MPC (Model Predictive Control) to drive a car around a track. 

## The Model

Student describes their model in detail. This includes the state, actuators and update equations.

The state of our model consists of following values:

* x: x coordinate of the car in the world coordinate system (in meters),
* y: y coordinate of the car in the world coordinate system (in meters),
* psi: yaw rate of the car in radians/per second,
* v: velocity of the car in m/s.

There are following actuators (control inputs) available:
* delta: steering angle of the front wheels, with the min/max range [-25°,25°],
* a: acceleration, which comprises also breaking (negative values). Range: [-1,1].

The update equations of the model are:
* x[t+1] = x[t] + v[t] * cos(psi[t]) * dt,
* y[t+1] = y[t] + v[t] * sin(psi[t]) * dt,
* psi[t+1] = psi[t] + v[t]/Lf * delta[t] * dt,
* v[t+1] = v[t] + a[t] * dt,
where 
* dt is the timespan between t+1 and t, 
* Lf is the distance in meters between the center of mass of the vehicle and it's front axle (in the project 2.67m).

## Timestep Length and Elapsed Duration (N & dt)
Following values have been chosen for the project:
* N = 10,
* dt = 0.1.

-> T = 1 second (N * dt)

"In the case of driving a car, T should be a few seconds, at most. Beyond that horizon, the environment will change enough that it won't make sense to predict any further into the future."

Several parameter combinations have been tried out during the development. Too high N led to very bad behaviour in the curves.

## Model Predictive Control with Latency
To deal with the given 0.1 seconds latency the state vector was fed with prediction values - where will the car be in 0.1 seconds (using the equations above, and the assumption psi = 0. See main.cpp, line 134-146). 
