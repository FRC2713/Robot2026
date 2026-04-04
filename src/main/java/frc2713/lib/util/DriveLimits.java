package frc2713.lib.util;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public record DriveLimits(
    LinearVelocity linearVelocity,
    LinearAcceleration linearAcceleration,
    AngularVelocity angularVelocity,
    AngularAcceleration angularAcceleration) {}
