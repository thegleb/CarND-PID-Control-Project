# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

## Overview

This project aims to control a car driving around a track using a pure PID controller.

The goal of a PID controller is to minimize what is called a cross track error (CTE).
In our case, the CTE is the car's distance from the center line of the road.
The PID value suggests the correction needed to bring the system (or in this case the car)
to a state where the CTE is minimized or 0. 

The formula for computing the final error is:
```
-tau_p * CTE - tau_d * diff_CTE - tau_i * int_CTE
```
where `tau_p` is the proportional coefficient, `tau_d` is the derivative coefficient, `tau_i` is the integral coefficient.

`diff_CTE` is computed by subtracting the previous `CTE` from the current value and `int_CTE` is the sum of the CTEs thus far.

#### What do `tau_p`, `tau_d` and `tau_i` *actually* do?

From manually and automatically tweaking the values, as well as reasoning about it, the way I understood these values is the following:

`tau_p` - this sets up a direct proportionality between the current CTE and the target (ideally 0). Increasing this
will increase the overshoot when the car tries to correct itself, assuming the other values are set to 0.
Practically speaking this determines how much the car will react to having a high CTE (higher steering angle).
Values that are too high will make the car oscillate more easily.

`tau_d` - this is basically a damping factor. This will suppress the overshoot set by a higher `tau_p` value. Increasing this
will determine how quickly the car's oscillation will be decreased. If the value increases too much, it actually causes oscillation
at a different frequency. `tau_d` mostly counteracts higher values of `tau_p`, and increasing one will probably require increasing the other
by some amount.

`tau_i` - the easiest way I understood it is that this determines how urgently the controller
reacts to a growing CTE, since this value multiplies the sum of all the cross-track errors. Increasing this value will
affect how quickly the car responds when it is going off the center line (absolute value of CTE is increasing).
This factor is most useful when the car is going around a corner, because this is when the CTE can grow rapidly,
as the car will tend to go off-course when it is moving quicker. 

#### Finding sane `tau_p`, `tau_d` and `tau_i` values

This is one part science, one part art, and one part dark magic.
I found some suggestions for coming up with these coefficients manually ([source](https://robotics.stackexchange.com/questions/167/what-are-good-strategies-for-tuning-pid-loops)):
```
* Set all gains to zero.
* Increase the P gain until the response to a disturbance is steady oscillation.
* Increase the D gain until the the oscillations go away (i.e. it's critically damped).
* Repeat steps 2 and 3 until increasing the D gain does not stop the oscillations.
* Set P and D to the last stable values.
* Increase the I gain until it brings you to the setpoint with
the number of oscillations desired (normally zero but a quicker response 
can be had if you don't mind a couple oscillations of overshoot)
```

I tried this a few times and came up with some reasonable `tau_p`, `tau_i`, and `tau_d` parameters for the a throttle value of 0.3,
something like `{0.2, 0.002, 10}`. The relative scales of each value were inspired partly by the second answer at the [link above](https://robotics.stackexchange.com/questions/167/what-are-good-strategies-for-tuning-pid-loops):

```
* Set all gains to 0.
* Increase Kd until the system oscillates.
* Reduce Kd by a factor of 2-4.
* Set Kp to about 1% of Kd.
* Increase Kp until oscillations start.
* Decrease Kp by a factor of 2-4.
* Set Ki to about 1% of Kp.
* Increase Ki until oscillations start.
* Decrease Ki by a factor of 2-4.
``` 

At this point with a reasonably working solution I wanted to see what it would take to run the car at a throttle value of 1.0.

Because tuning manually was not very time-efficient, I implemented a version of the twiddle algorithm 
described in the lessons (code comment from `main.cpp` reproduced here):

```
- for each multiplier (Tp, Ti, and Td), adjust it up by 1x unit or down by 1x unit
  (where each unit can be an arbitrary value)
-- if this change improved error, then increase the unit for this factor by 10%
-- if it did NOT improve error, then decrease the unit for this factor by 10%
- repeat until the sum of the units is below some threshold (or some other arbitrary goal post)
```

With this algorithm I noticed a couple things. One, I had to allow the algorithm to run for
most of the track, because the values that worked well for the first 1-2 corners would break down on corners 3 and 4.
Two, with "bad" values (or actually most values) the car would actually fly off the track and either get stuck or spend
a bunch of time running in circles off-track.

To save time and fix these problems I set the number of total steps to evaluate to around 1500 which would cover
the vast majority of the track, and also added a kill switch for when the car either leaves the track
(absolute value of cross track error crosses 4.5) or if the car slows down to < 50% of its theoretical
top speed for the throttle value. When this condition is hit, the error is set to a high value
to make sure this state is not encouraged.

```
// if we have at least passed the initial few iterations but either:
// - hit something that causes speed to drop
// - go off-track (abs(cte) > 4.5)
// then we should abort the cycle early
if (
    (num_iterations > num_iterations_to_ignore && speed < max_throttle_val * 100 / 2) ||
    (num_iterations > 100 && abs(cte_steer) > 4.5)
    ) {
    abort_early = true;
}
...
if (abort_early) {
    std::cout << "aborted early" << std::endl;
    total_err = 1000;
    abort_early = false;
}
```

After letting the optimizer algorithm run for many generations, I got some P, I, and D value candidates,
and quickly realized that even using these values directly wasn't reliable, as the "best" values would still fling the
car off-track at random.

Getting the car to stop falling off the track at a throttle of 1.0 on *every* corner was difficult,
so I picked the last good parameter values I liked and instead decreased throttle to 0.75.
I also manually tweaked the P, I, and D values further based on my understanding of what each value affected.

Finally, I added an extra if/else clause that made sure that if the CTE was increasing,
we decreased throttle, and if the CTE was decreasing, then we increased the throttle again
again. I also set hard limits to the minimum/maximum throttle value to make sure the
car would never stop or exceed the limits:

```
if (abs(cte_steer) > prev_err && prev_err > 0) {
    throttle_val -= 0.009;

    if (throttle_val < max_throttle_val / 2) {
        throttle_val = max_throttle_val / 2;
    }
} else {
    throttle_val += 0.01;
    
    if (throttle_val > max_throttle_val) {
        throttle_val = max_throttle_val;
    }
}
prev_err = abs(cte_steer);
```

The final code's driving looks something like this:

[![PID controller, only slightly drunk](https://img.youtube.com/vi/B7zuMOKWpAM/0.jpg)](https://www.youtube.com/watch?v=B7zuMOKWpAM)

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)