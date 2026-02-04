# ROBOT MOTION ENGINE

The Robot Motion Engine is a software framework for managing, planning, and controlling robot motion.
It provides a modular architecture that separates robot management, motion planning, scheduling, and control.

---

## Classes

### RobotManager

The `RobotManager` class is responsible for initializing and managing robot instances based on configuration files.
It serves as the entry point for setting up and running robot motion.

- **load**
    : Reads a `YAML` configuration file and maps its contents to a custom configuration structure.
      This configuration typically includes DH parameters, limits and motion options.

- **start**
    : Starts the robot motion system.
      If the robot is not located at the home position, it first moves the robot to the home position before executing any motion.

- **stop**
    : Stops the robot motion safely and commands the robot to return to the home position.

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
