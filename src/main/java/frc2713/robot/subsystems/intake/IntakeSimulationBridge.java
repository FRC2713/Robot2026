package frc2713.robot.subsystems.intake;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

/**
 * Bridges maple-sim IntakeSimulation with the robot's IntakeRoller. Syncs start/stop with roller
 * state and provides game piece count for launch integration.
 */
public class IntakeSimulationBridge {
  private final IntakeSimulation intakeSimulation;
  private final IntakeRoller intakeRoller;

  public IntakeSimulationBridge(SwerveDriveSimulation driveSimulation, IntakeRoller intakeRoller) {
    this.intakeRoller = intakeRoller;
    this.intakeSimulation =
        IntakeSimulation.OverTheBumperIntake(
            "Fuel",
            driveSimulation,
            IntakeConstants.Extension.width,
            IntakeConstants.Extension.extendedPosition.get(),
            IntakeConstants.Sim.intakeSide,
            IntakeConstants.Sim.capacity);
    intakeSimulation.register(SimulatedArena.getInstance());
  }

  /** Syncs intake simulation state with roller. Call from simulation periodic. */
  public void update() {
    if (intakeRoller.isIntaking()) {
      intakeSimulation.startIntake();
    } else {
      intakeSimulation.stopIntake();
    }
  }

  /** Returns true if there is at least one game piece in the intake. */
  public boolean hasGamePiece() {
    return intakeSimulation.getGamePiecesAmount() > 0;
  }

  /**
   * Removes one game piece from the intake. Call when launching.
   *
   * @return true if a game piece was removed
   */
  public boolean obtainGamePiece() {
    return intakeSimulation.obtainGamePieceFromIntake();
  }
}
