# ROBOT MOTION ENGINE

---

## Classes

### RobotManager
The RobotManager class manages robots initialized from configuration file.

#### functions

- load

- start

- stop

___

### RobotController
The RobotController class controls a robot to achive generated motions.

#### functions

- load

- home

- start

- stop

___

### Scheduler
The Scheduer class manages sequence of actions of the robot. The actions is represnted as states.
We used to concept FSM (Finite State Machine).

#### functions

- reset

- tick

___

### Planner
The Planner class generates motions or path required at the state.

#### functions

- reset

- eval

___

### Controller
The Controller class compute control input to achive the trajectory.

#### functions

- reset

- compute

***