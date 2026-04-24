package frc2713.robot.subsystems.serializer;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc2713.lib.io.ArticulatedComponent;
import frc2713.lib.io.MotorIO;
import frc2713.lib.io.MotorInputsAutoLogged;
import frc2713.lib.logging.PeriodicTimingLogger;
import frc2713.lib.logging.TimeLogged;
import frc2713.lib.subsystem.MotorSubsystem;
import frc2713.lib.subsystem.TalonFXSubsystemConfig;
import frc2713.robot.subsystems.launcher.LaunchingLookupMaps;
import frc2713.robot.subsystems.launcher.LaunchingSolutionManager;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class DyeRotor extends MotorSubsystem<MotorInputsAutoLogged, MotorIO>
    implements ArticulatedComponent {

  public DyeRotor(final TalonFXSubsystemConfig config, final MotorIO indexerMotorIO) {
    super(config, new MotorInputsAutoLogged(), indexerMotorIO);

    setDefaultCommand(setVelocity(() -> RPM.of(0)));
  }

  public Command setVelocity(Supplier<AngularVelocity> desiredVelocity) {
    return velocitySetpointCommand(desiredVelocity);
  }

  public Command indexFuel() {
    return setVelocity(SerializerConstants.DyeRotor.indexingSpeed);
  }

  @AutoLogOutput(key = "Dye Rotor/DynamicIndexSpeed")
  public AngularVelocity dynamicIndexSpeed() {
    return RPM.of(
        LaunchingLookupMaps.distanceToDyeRotorSpeedMap.get(
            LaunchingSolutionManager.getInstance().getSolution().effectiveDistanceMeters()));
  }

  @AutoLogOutput(key = "Dye Rotor/DynamicIndexScaleFactor")
  public double dynamicIndexScaleFactor(BooleanSupplier adjustSpeedForOtf) {
    return adjustSpeedForOtf.getAsBoolean()
        ? SerializerConstants.DyeRotor.indexingOTFScaleFactor.get()
        : 1.0;
  }

  public Command dynamicIndex() {
    return dynamicIndex(() -> false);
  }

  public Command dynamicIndex(BooleanSupplier adjustSpeedForOtf) {
    return setVelocity(() -> this.dynamicIndexSpeed().times(this.dynamicIndexScaleFactor(adjustSpeedForOtf)));
  }

  public Command stirFuel() {
    return setVelocity(SerializerConstants.DyeRotor.stirSpeed);
  }

  public Command dynamicFeedWhenReady(BooleanSupplier isReady, BooleanSupplier adjustSpeedForOtf) {
    return Commands.sequence(Commands.waitUntil(isReady), dynamicIndex(adjustSpeedForOtf));
  }

  public Command feedWhenReady(BooleanSupplier isReady) {
    return feedWhenReady(isReady, Seconds.of(Double.POSITIVE_INFINITY));
  }

  public Command feedWhenReady(BooleanSupplier isReady, Time timout) {
    return Commands.sequence(Commands.waitUntil(isReady).withTimeout(timout), indexFuel());
  }

  public Command outtakeFuel() {
    return setVelocity(() -> SerializerConstants.DyeRotor.outdexingSpeed);
  }

  public Command stop() {
    return setVelocity(() -> RotationsPerSecond.of(0));
  }

  @Override
  @TimeLogged("Performance/SubsystemPeriodic/DyeRotor")
  public void periodic() {
    try (var ignored = PeriodicTimingLogger.time(this)) {
      super.periodic();
      // Additional periodic code for indexer can be added here
    }
  }

  @Override
  public Transform3d getTransform3d() {
    Angle rotations = super.getCurrentPosition();
    Transform3d localTransform =
        new Transform3d(new Translation3d(), new Rotation3d(0, 0, rotations.in(Radians)));

    return config.initialTransform.plus(localTransform);
  }
}
