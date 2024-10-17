# dymp_mpc

## Abstract

dymp_mpc is a C++ program that implements a model predictive controller of legged and humanoid robots.
The [dymp](https://github.com/ytazz/dymp) C++ library is used for core trajectory optimization.
Currently dymp_mpc provides implementation of a controller as SimpleController of [Choreonoid](https://github.com/choreonoid).

## Building

Use CMake.

### Dependent libraries

- [dymp](https://github.com/ytazz/dymp)
- [vnoid][dymp](https://github.com/ytazz/vnoid)
- [Choreonoid](https://github.com/choreonoid)

## Running

T.B.D.

## Related paper

The centroidal dynamics model implemented in dymp is described in the following paper:
[Y. Tazaki, "Trajectory Generation for Legged Robots Based on a Closed-Form Solution of Centroidal Dynamics," in IEEE Robotics and Automation Letters, vol. 9, no. 11, pp. 9239-9246, Nov. 2024]
(https://ieeexplore.ieee.org/abstract/document/10669176).

The whole-body dynamics model is described in the following one:
[Humanoid Dance Simulation Using Choreonoid and Whole-Body Model Predictive Control]
(https://atompc-workshop.github.io/assets/pdf/paper2.pdf).

