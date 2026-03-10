// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc2713.robot.subsystems.drive;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.Arrays;
import java.util.Queue;

/** IO implementation for NavX. */
public class GyroIONavX implements GyroIO {
  private final AHRS navX = new AHRS(NavXComType.kMXP_SPI, (byte) Drive.ODOMETRY_FREQUENCY);
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  public GyroIONavX() {
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(navX::getYaw);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = navX.isConnected();
    inputs.yawPosition = Rotation2d.fromDegrees(-navX.getYaw());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(-navX.getRawGyroZ());

    inputs.odometryYawTimestamps = drainDoubleQueue(yawTimestampQueue);
    inputs.odometryYawPositions = drainYawQueue(yawPositionQueue);
  }

  private static double[] drainDoubleQueue(Queue<Double> queue) {
    int size = queue.size();
    double[] values = new double[size];
    int count = 0;
    while (count < size) {
      Double value = queue.poll();
      if (value == null) {
        break;
      }
      values[count++] = value;
    }
    return (count == size) ? values : Arrays.copyOf(values, count);
  }

  private static Rotation2d[] drainYawQueue(Queue<Double> queue) {
    int size = queue.size();
    Rotation2d[] values = new Rotation2d[size];
    int count = 0;
    while (count < size) {
      Double value = queue.poll();
      if (value == null) {
        break;
      }
      values[count++] = Rotation2d.fromDegrees(-value);
    }
    return (count == size) ? values : Arrays.copyOf(values, count);
  }
}
