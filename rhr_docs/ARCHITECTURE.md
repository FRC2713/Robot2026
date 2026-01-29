# Subsystems

* Subsystems should contain an action. i.e. one subsystem for the pivot or intake rollers where on, one for the intake rollers

## Folders

* \[SUBSYSTEM\_NAME\]  
* \[SUBSYSTEM\_NAME\]IO  
  *   
* \[SUBSYSTEM\_NAME\]IO\[IO\_IMPLEMENTATION\]  
* \[SUBSYSTEM\_NAME\]Constants  
  * Whenever possible extend generic  
  * [See general constant rules](#constants)

## Command Patterns

* Commands patterns we've used in the past:
  * atomic commands (e.g `InstantCommands`) are defined as class methods in their Subsystems
  * more complex commands are defined in their own .java in \[SUBSYSTEM\_NAME\]Commands
  * subsystems that are defined as state machines have commands handle transitioning from state to state ([example](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/subsystems/intake/Intake.java))

# Constants {#constants}

* Use SI WPI units wherever possible  
* No “k” prefix

# Abstract Classes

* As a goal, we’ll attempt to create abstract implementations of common subsystems  
  * Revolute Joint  
  * Linear Joint  
  * Flywheel  
  * Etc

# State Machines

* Consider state machines to manage subsystem states.  
* If necessary, add a state machine to manage the state of the entire robot.  
* [https://github.com/Team254/FRC-2025-Public/blob/main/src/main/java/com/team254/frc2025/subsystems/superstructure/SuperstructureStateMachine.java](https://github.com/Team254/FRC-2025-Public/blob/main/src/main/java/com/team254/frc2025/subsystems/superstructure/SuperstructureStateMachine.java)  
* [https://github.com/stateless4j/stateless4j](https://github.com/stateless4j/stateless4j)

# Operator Interface

* Separate classes for separate controllers  
  * Driver controller, Operator controller, and developer controller.  And more
