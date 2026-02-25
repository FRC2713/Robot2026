package frc2713.lib.subsystem;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.DifferentialMotorConstants;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc2713.lib.drivers.CANDeviceId;

public class DifferentialSubsystemConfig {

  public String name = "UNNAMED";
  public CANDeviceId leaderCANID;
  public CANDeviceId followerCANID;

  // Leader and follower TalonFX configurations
  public TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
  public TalonFXConfiguration followerConfig = new TalonFXConfiguration();

  // Differential mechanism configuration
  public DifferentialMotorConstants<TalonFXConfiguration> differentialConstants;

  // Marks Slot0 and Slot1 as live-tunable
  public boolean tunable = false;
  public boolean useFOC = true;

  // Differential-specific ratios
  // Ratio of average rotor to units for this differential mechanism
  public double averageGearRatio = 1.0;
  // Ratio of difference rotor to units for this differential mechanism
  public double differenceGearRatio = 1.0;
  // The number of times the mechanism has to rotate to achieve 1m of travel
  public double unitRotationsPerMeter = 1.0;
  public double kMinPositionUnits = Double.NEGATIVE_INFINITY;
  public double kMaxPositionUnits = Double.POSITIVE_INFINITY;

  // PID gains for average axis (Slot0) and difference axis (Slot1)
  public Slot0Configs averageGains = new Slot0Configs();
  public Slot1Configs differenceGains = new Slot1Configs();

  // Motor alignment
  public MotorAlignmentValue motorAlignment = MotorAlignmentValue.Aligned;

  // Closed loop update rate
  public double closedLoopRate = 200.0;

  // Whether follower uses common leader configs
  public boolean followerUsesCommonLeaderConfigs = true;

  // Moment of Inertia (KgMetersSquared) for sim
  public double momentOfInertia = 0.5;

  // Physical transform for 3D visualization
  public Transform3d initialTransform =
      new Transform3d(new Translation3d(0, 0, Inches.of(18.484119).in(Meters)), new Rotation3d());
}
