package frc2713.lib.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class WaitSupplierCommand extends Command {
  protected Timer m_timer = new Timer();
  public double m_duration;

  private final DoubleSupplier timeSupplier;

  public WaitSupplierCommand(DoubleSupplier timeSupplier) {
    this.timeSupplier = timeSupplier;
  }

  @Override
  public void initialize() {
    m_duration = timeSupplier.getAsDouble();
    m_timer.restart();
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_duration);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
