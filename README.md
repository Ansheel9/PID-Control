# PID Controller
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

---

Udacity Self-Driving Car Engineer Nanodegree

Project 8

---

## Introduction

![Result](./images/track_one.gif)

The PID (Proportional, Integral and Derivative) controller is a closed loop controller widely used by the industry. It computes the system input variable from the error e(t) between the desired set point and the system output (process variable). The control response (system input) is calculated by applying the proportional, derivative and integral gains over e(t).

This project consists of a c++ implementation of PID controller to control the steering angle of a car using the Udacity simulator. The main goals of this project is to develop a c++ PID controller that successfully drives the vehicle around the track (Udacity simulator).

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

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.

## PID

For this project, the PID controller is used to control the steering angle from the ``cross track error`` e(t) (car distance from the track center). Therefore, the system input is the steering angle, the output is the car distance from the center and the setpoin it zero (closest to the center as possible). 

![equation](http://latex.codecogs.com/gif.latex?%5Calpha%20%3D%20-K_pe%28t%29%20-K_d%5Cfrac%7Bde%28t%29%7D%7Bdt%7D%20-%20K_i%5Csum%20e%28t%29)

#### Kp (Proportional Gain)

The Kp gain results into a proportional control response. In the context of this project, it means that the steer input is in proportion to the Cross Track error. However, the proportional control alone results into a marginally stable system because the car will never converge to the set point, it will slightly overshoot. In addition, increasing the value of Kp will make the car react faster but it will also oscillate more around the center lane (set point). This [video](https://github.com/otomata/CarND-Controls-PID/blob/master/images/kp.mp4) shows the oscillation of the car trajectory using a proportional controller. 

#### Kd (Derivative Gain)

The Kd gain considers the rate change of error and tries to bring this rate to zero. The derivative gain complements the proportional output by reducing the overshoot, it mains goal is to flattening the car trajectory. This [video](https://github.com/otomata/CarND-Controls-PID/blob/master/images/kd.mp4) presents the smoothed trajectory of the car using a proportional-derivative controller. 


#### Ki (Integral Gain)

The Ki gain reduces the persistent error, i.e. the accumulated error. The integral gain helps the controller to deal with the  ``systematic bias`` problem witch leads to a systematic error. The Ki gain should help to remove the residual error to approximate the car near to the center of the track. This [video](https://github.com/otomata/CarND-Controls-PID/blob/master/images/ki.mp4) illustrates the car following the lane center using a complete PID controller. It is important to mention that the car oscillates a little bit more than PD controller because the ki gain has been manually tuned. Next section, we presented the twiddle algorithm we used to fine tuning our PID controller. 

