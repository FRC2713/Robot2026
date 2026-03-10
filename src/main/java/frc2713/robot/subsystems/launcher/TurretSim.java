package frc2713.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import frc2713.lib.io.CanCoderInputsAutoLogged;
import frc2713.lib.io.SimCanCoderIO;
import frc2713.lib.io.SimTalonFXIO;

public class TurretSim extends Turret {

  public TurretSim() {
    this(new SimTalonFXIO(LauncherConstants.Turret.config));
  }

  private TurretSim(SimTalonFXIO turretMotorIO) {
    super(
        LauncherConstants.Turret.config,
        turretMotorIO,
        new CanCoderInputsAutoLogged(),
        new SimCanCoderIO(
            LauncherConstants.Turret.canCoderConfig,
            () -> {
              var state = new SimCanCoderIO.SimCanCoderState();
              double turretRotations = turretMotorIO.getSimMechanismPositionRotations();
              double turretRps = turretMotorIO.getSimMechanismVelocityRps();
              state.position =
                  Rotations.of(turretRotations * LauncherConstants.Turret.encoderToTurretGearRatio);
              state.velocity =
                  RotationsPerSecond.of(
                      turretRps * LauncherConstants.Turret.encoderToTurretGearRatio);
              return state;
            }));
  }
}
