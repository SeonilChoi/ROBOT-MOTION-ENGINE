# ROBOT MOTION ENGINE

This project provides a framework for managing and controlling robot motion.

---

## Classes

### RobotManager

The `RobotManager` class manages robots initialized from configuration files.

- load
- start
- stop

### RobotController

The `RobotController` class controls a robot to execute generated motions.

- load
- home
- start
- stop

### Scheduler

The `Scheduer` class manages a sequence of robot actions.
Actions are represnted as states, following the concept of an FSM (Finite State Machine).

- reset
- tick

### Planner

The `Planner` class generates motions or paths required for each state.

- reset
- eval

### Controller

The `Controller` class computes control input to track the desired trajectory.

- reset
- compute
