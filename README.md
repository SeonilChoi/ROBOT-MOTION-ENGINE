# ROBOT MOTION ENGINE

---

## Classes

### RobotManager
The RobotManager class manages robots initialized from configuration file.

#### functions

- load

- start

- stop

### RobotController
The RobotController class controls a robot to achive generated motions.

#### functions

- load

- home

- start

- stop

### Scheduler
The Scheduer class manages sequence of actions of the robot. The actions is represnted as states.
We used to concept FSM (Finite State Machine).

#### functions

- reset

- tick

### Planner
The Planner class generates motions or path required at the state.

#### functions

- reset

- eval

### Controller
The Controller class compute control input to achive the trajectory.

#### functions

- reset

- compute