# Subsystems & Mechanisms

Subsystem folders should contain related mechanisms. For example, an `intake` subsystem folder could contain a four-bar mechanism and a roller mechanism.
> [!NOTE]
> Note the difference in terminology between WPILib subsystems and as they are documented here and by the CAD team. We group different WPILib "subsystems" for simplicity. All mechanisms should still at some point extend WPILib's `SubsystemBase`.

## Layout
```
[subsystem] # The subsystem foloder
├── [Subsystem]Constants.java # Constants for the mechanisms.
├── [Subsystem][Mechanism_1].java # Mechanism
├── [Subsystem][Mechanism_1]IO[Implementation].java # Mechanism IO implementation (if applicable)
└── [Subsystem][Mechanism_2].java # Another mechanism
```
For example,
```
intake
├── IntakeConstants.java
├── IntakeExtension.java
├── IntakeExtensionIOTalonFX.java
└── IntakeRoller.java
```

## Command Patterns

* Commands patterns we've used in the past:
  * atomic commands (e.g `InstantCommands`) are defined as class methods in their Subsystems
  * more complex commands are defined in their own .java in \[SUBSYSTEM\_NAME\]Commands
  * subsystems that are defined as state machines have commands handle transitioning from state to state ([example](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/src/main/java/org/littletonrobotics/frc2026/subsystems/intake/Intake.java))

# Constants

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
