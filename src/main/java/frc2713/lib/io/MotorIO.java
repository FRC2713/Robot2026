package frc2713.lib.io;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.*;
import frc2713.lib.drivers.CANDeviceId;

public interface MotorIO {
  default void readInputs(MotorInputs inputs) {}

  default void setOpenLoopDutyCycle(double dutyCycle) {}

  default void setPositionSetpoint(Angle setpoint) {}

  default void setMotionMagicSetpoint(Angle setpoint) {
    setMotionMagicSetpoint(setpoint, 0);
  }

  default void setMotionMagicSetpoint(Angle setpoint, int slot) {}

  default void setMotionMagicSetpoint(
      Angle setpoint,
      AngularVelocity velocity,
      AngularAcceleration acceleration,
      Velocity<AngularAccelerationUnit> jerk) {
    setMotionMagicSetpoint(setpoint, velocity, acceleration, jerk, 0);
  }

  default void setMotionMagicSetpoint(
      Angle setpoint,
      AngularVelocity velocity,
      AngularAcceleration acceleration,
      Velocity<AngularAccelerationUnit> jerk,
      int slot) {
    setMotionMagicSetpoint(setpoint, velocity, acceleration, jerk, slot, 0.0);
  }

  default void setMotionMagicSetpoint(
      Angle setpoint,
      AngularVelocity velocity,
      AngularAcceleration acceleration,
      Velocity<AngularAccelerationUnit> jerk,
      int slot,
      double feedforward) {}

  default void setNeutralMode(NeutralModeValue mode) {}

  default void setVelocitySetpoint(AngularVelocity unitsPerSecond) {
    setVelocitySetpoint(unitsPerSecond, 0);
  }

  default void setVelocitySetpoint(AngularVelocity unitsPerSecond, int slot) {}

  default void setVoltageOutput(Voltage voltage) {}

  default void setCurrentPositionAsZero() {}

  default void setCurrentPosition(Angle position) {}

  default void setEnableSoftLimits(boolean forward, boolean reverse) {}

  default void setEnableHardLimits(boolean forward, boolean reverse) {}

  default void setEnableAutosetPositionValue(boolean forward, boolean reverse) {}

  default void follow(CANDeviceId leaderId, boolean opposeLeaderDirection) {}

  default void setTorqueCurrentFOC(Current current) {}

  default void setMotionMagicConfig(MotionMagicConfigs config) {}

  default void setVoltageConfig(VoltageConfigs config) {}

  default double getClosedLoopError() {
    return 0.0;
  }

  default boolean isMagicMotionAtTarget() {
    return true;
  }
}
