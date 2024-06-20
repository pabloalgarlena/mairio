# Installation


```bash
sudo apt-get install libsdl2-dev libsdl2-image-dev libsdl2-mixer-dev libsdl2-ttf-dev libfreetype6-dev libportmidi-dev libjpeg-dev build-essential python3-setuptools python3-dev python3-numpy python3-pip python3-virtualenv cython3 python3-full swig
```

# Action space

Discrete:
 - steer left
 - steer center
 - steer right
 - forward
 - stop
 - backward

# Observation space

- size of the ball in pixels
- horizontal position of the ball

# Rewards

 Based on how close the ditance is to the target distance and how centered the ball is

# Starting state

The car starts at the center of a box with the ball in front of it at the target distance
A spline is generated for the ball to move.
The spline is defined by 7 control vectors, randomly positioned and with random speed between 0.5 and 1.0 m/s, the first and last of them being 0m/s.

# Episode termination

The episode terminates qhen the splines finishes or the car exists the box