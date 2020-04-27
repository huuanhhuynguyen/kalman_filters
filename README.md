# Extended and Unscented Kalman Filters

This project is based on the [Sensor Fusion section](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013) of the Self-Driving Car Engineer Nanodegree from Udacity.
Even I didn't register for the course, I learned EKF & UKF from reading articles
 to this Nanodegree. Thanks Udacity for giving me this learning experience!

## Project Setting
Radar and Lidar measurements as well as ground-truth are stored in text files 
(in ./data). Kalman Filters are used to fuse the measurements from two sensors 
and result in the estimations.

## Fusion Model

Lidar KF and Radar KF process the Lidar and Radar measurement, respectively.

![Fusion Model](out/fusion.png)

To fuse the estimation, the state X and covariance matrix P are shared between 
two filters. It is also possible to share the process uncertainty Q in addition 
to X and P.

## Linear KF

It is possible to use only linear KFs for fusing the measurements. In that case,
the state X = [x, y, vx, vy] and the measurement for both sensor z = [x, y].

For it, just extract the position measurement from each sensor. But to make use
of velocity measurement from Radar, EKF and UKF could be used because a 
non-linearity now presents.

## Extended KF

I use a linear KF for processing the Lidar position measurement. That means,
X = [x, y, vx, vy] and z = [x, y]
I use an EKF for processing the Radar position measurement. That means,
X = [x, y, vx, vy] and z = [rho, phi, rho_dot]

| 1.txt | 2.txt | 3.txt |
| --- | --- | --- |
| ![1](out/1.png) | ![2](out/2.png) | ![3](out/3.png)|

The images above show the result. I haven't tune the uncertainty matrices yet. 
P, Q and R are simply identity matrices.

## Unscented KF

## Basic Build Instructions
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./kalman_filters_cpp`
TODOS run with args
 

