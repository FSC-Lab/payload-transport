# Multirotor Slung-Load-Transport-System Real-Time Controller Implementation

This is the repository for re-implementation of Longhao Qian's controllers for Multirotor Slung Load Transport Systems.

## Building the code

This code is written with ROS 2 compatibility in mind, and thus is built with `colcon`.

``` bash
colcon build --merge-install
```

## Originating Literature

* Dynamics and control of a quadrotor with a cable suspended payload
  * **Modelled the system dynamics** of a single-vehicle slung load transport system and designed basic translational motion controller

``` bibtex
@inproceedings{qian2017dynamics,
  title={Dynamics and control of a quadrotor with a cable suspended payload},
  author={Qian, Longhao and Liu, Hugh HT},
  booktitle={2017 IEEE 30th Canadian Conference on Electrical and Computer Engineering (CCECE)},
  pages={1--4},
  year={2017},
  organization={IEEE}
}
```

* Path-following control of a quadrotor UAV with a cable-suspended payload under wind disturbances
  * Designed a **UDE-based path-following controller** for a single-vehicle slung load transport system operating under wind disturbanes

``` bibtex
@article{qian2019path,
  title={Path-following control of a quadrotor UAV with a cable-suspended payload under wind disturbances},
  author={Qian, Longhao and Liu, Hugh HT},
  journal={IEEE Transactions on Industrial Electronics},
  volume={67},
  number={3},
  pages={2021--2029},
  year={2019},
  publisher={IEEE}
}
```

* Path following control of multiple quadrotors carrying a rigid-body slung payload
  * Modelled the system dynamics of a **multi-vehicle slung load transport system** and designed a path-following controller for which

``` bibtex
@inproceedings{qian2019path,
  title={Path following control of multiple quadrotors carrying a rigid-body slung payload},
  author={Qian, Longhao and Liu, Hugh H},
  booktitle={AIAA Scitech 2019 Forum},
  pages={1172},
  year={2019}
}
```
