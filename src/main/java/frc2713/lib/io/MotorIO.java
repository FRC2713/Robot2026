package frc2713.lib.io;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.*;
import frc2713.lib.drivers.CANDeviceId;

public interface MotorIO {
  void readInputs(MotorInputs inputs);

  void setOpenLoopDutyCycle(double dutyCycle);

  void setPositionSetpoint(Angle setpoint);

  default void setMotionMagicSetpoint(Angle setpoint) {
    setMotionMagicSetpoint(setpoint, 0);
  }

  void setMotionMagicSetpoint(Angle setpoint, int slot);

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

  void setMotionMagicSetpoint(
      Angle setpoint,
      AngularVelocity velocity,
      AngularAcceleration acceleration,
      Velocity<AngularAccelerationUnit> jerk,
      int slot,
      double feedforward);

  void setNeutralMode(NeutralModeValue mode);

  default void setVelocitySetpoint(AngularVelocity unitsPerSecond) {
    setVelocitySetpoint(unitsPerSecond, 0);
  }

  void setVelocitySetpoint(AngularVelocity unitsPerSecond, int slot);

  void setVoltageOutput(Voltage voltage);

  void setCurrentPositionAsZero();

  void setCurrentPosition(Angle position);

  void setEnableSoftLimits(boolean forward, boolean reverse);

  void setEnableHardLimits(boolean forward, boolean reverse);

  void setEnableAutosetPositionValue(boolean forward, boolean reverse);

  void follow(CANDeviceId leaderId, boolean opposeLeaderDirection);

  void setTorqueCurrentFOC(Current current);

  void setMotionMagicConfig(MotionMagicConfigs config);

  void setVoltageConfig(VoltageConfigs config);
}
