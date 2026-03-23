package frc2713.lib.energy;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Joules;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Watts;

import edu.wpi.first.wpilibj.Timer;
import frc2713.lib.io.AdvantageScopePathBuilder;
import java.util.HashMap;
import java.util.Map;
import java.util.TreeMap;
import org.littletonrobotics.junction.Logger;

public class EnergyManagement {

  public static class EnergyMonitor {

    private static class Holder {
      private static final EnergyMonitor INSTANCE = new EnergyMonitor();
    }

    private static final String BASE_PATH = "EnergyMonitor";

    private final Map<String, MotorEnergyState> motorStates = new HashMap<>();

    private EnergyMonitor() {}

    public static EnergyMonitor getInstance() {
      return Holder.INSTANCE;
    }

    /**
     * Report voltage and current for a motor. Called by subsystems from their periodic().
     *
     * @param subsystemName Subsystem name (e.g. "Drive", "Flywheels")
     * @param motorName Motor identifier (e.g. "Motor", "Leader", "Module0_Drive")
     * @param voltageV Applied voltage in volts
     * @param currentA Current in amps (supply current preferred for battery-side energy)
     */
    public static void report(
        String subsystemName, String motorName, double voltageV, double currentA) {
      getInstance().reportInternal(subsystemName, motorName, voltageV, currentA);
    }

    private void reportInternal(
        String subsystemName, String motorName, double voltageV, double currentA) {
      String key = subsystemName + "/" + motorName;
      MotorEnergyState state = motorStates.get(key);
      if (state == null) {
        state = new MotorEnergyState(BASE_PATH + "/" + subsystemName + "/" + motorName);
        motorStates.put(key, state);
      }

      double powerW = voltageV * currentA;
      double nowSec = Timer.getFPGATimestamp();

      state.powerW = powerW;
      state.voltageV = voltageV;
      state.currentA = currentA;

      double dtSec = nowSec - state.lastReportTimeSec;
      if (state.lastReportTimeSec > 0) {
        state.totalEnergyJ += powerW * dtSec;
        if (powerW > 0) {
          state.totalEnergyDrawJ += powerW * dtSec;
        } else if (powerW < 0) {
          state.totalEnergyRegeneratedJ += -powerW * dtSec;
        }
      }
      state.lastReportTimeSec = nowSec;

      if (powerW > state.peakPowerW) {
        state.peakPowerW = powerW;
      }
      if (currentA > state.peakCurrentA) {
        state.peakCurrentA = currentA;
      }
    }

    /** Flush all motor data to AdvantageKit. Call once per cycle from Robot.robotPeriodic(). */
    public static void log() {
      getInstance().logInternal();
    }

    private void logInternal() {
      // Per-motor logging
      for (MotorEnergyState state : motorStates.values()) {
        Logger.recordOutput(state.pb.makePath("PowerW"), state.powerW, Watts);
        Logger.recordOutput(state.pb.makePath("VoltageV"), state.voltageV, Volts);
        Logger.recordOutput(state.pb.makePath("CurrentA"), state.currentA, Amps);
        Logger.recordOutput(state.pb.makePath("TotalEnergyJ"), state.totalEnergyJ, Joules);
        Logger.recordOutput(state.pb.makePath("TotalEnergyDrawJ"), state.totalEnergyDrawJ, Joules);
        Logger.recordOutput(
            state.pb.makePath("TotalEnergyRegeneratedJ"), state.totalEnergyRegeneratedJ, Joules);
        Logger.recordOutput(state.pb.makePath("PeakPowerW"), state.peakPowerW, Watts);
        Logger.recordOutput(state.pb.makePath("PeakCurrentA"), state.peakCurrentA, Amps);
      }

      // Per-subsystem aggregation (sum of motors in each subsystem)
      Map<String, AggregatedState> bySubsystem = new TreeMap<>();
      for (Map.Entry<String, MotorEnergyState> e : motorStates.entrySet()) {
        String subsystemName = e.getKey().substring(0, e.getKey().indexOf('/'));
        MotorEnergyState m = e.getValue();
        bySubsystem
            .computeIfAbsent(subsystemName, k -> new AggregatedState())
            .add(m.powerW, m.totalEnergyJ, m.totalEnergyDrawJ, m.totalEnergyRegeneratedJ,
                m.peakPowerW, m.peakCurrentA);
      }

      // Robot-wide aggregation (sum of all subsystems)
      AggregatedState robot = new AggregatedState();
      for (AggregatedState sub : bySubsystem.values()) {
        robot.add(sub.powerW, sub.totalEnergyJ, sub.totalEnergyDrawJ, sub.totalEnergyRegeneratedJ,
            sub.peakPowerW, sub.peakCurrentA);
      }

      // Log per-subsystem totals
      for (Map.Entry<String, AggregatedState> e : bySubsystem.entrySet()) {
        String pathBase = BASE_PATH + "/" + e.getKey() + "/Total";
        logAggregated(pathBase, e.getValue());
      }

      // Log robot totals
      logAggregated(BASE_PATH + "/Robot", robot);
    }

    private void logAggregated(String pathBase, AggregatedState a) {
      AdvantageScopePathBuilder pb = new AdvantageScopePathBuilder(pathBase);
      Logger.recordOutput(pb.makePath("PowerW"), a.powerW, Watts);
      Logger.recordOutput(pb.makePath("TotalEnergyJ"), a.totalEnergyJ, Joules);
      Logger.recordOutput(pb.makePath("TotalEnergyDrawJ"), a.totalEnergyDrawJ, Joules);
      Logger.recordOutput(pb.makePath("TotalEnergyRegeneratedJ"), a.totalEnergyRegeneratedJ, Joules);
      Logger.recordOutput(pb.makePath("PeakPowerW"), a.peakPowerW, Watts);
      Logger.recordOutput(pb.makePath("PeakCurrentA"), a.peakCurrentA, Amps);
    }

    private static final class AggregatedState {
      double powerW;
      double totalEnergyJ;
      double totalEnergyDrawJ;
      double totalEnergyRegeneratedJ;
      double peakPowerW;
      double peakCurrentA;

      void add(double pW, double eJ, double drawJ, double regenJ, double peakP, double peakI) {
        powerW += pW;
        totalEnergyJ += eJ;
        totalEnergyDrawJ += drawJ;
        totalEnergyRegeneratedJ += regenJ;
        peakPowerW = Math.max(peakPowerW, peakP);
        peakCurrentA = Math.max(peakCurrentA, peakI);
      }
    }

    private static final class MotorEnergyState {
      final AdvantageScopePathBuilder pb;
      double powerW;
      double voltageV;
      double currentA;
      double totalEnergyJ;
      double totalEnergyDrawJ;
      double totalEnergyRegeneratedJ;
      double peakPowerW;
      double peakCurrentA;
      double lastReportTimeSec;

      MotorEnergyState(String basePath) {
        this.pb = new AdvantageScopePathBuilder(basePath);
      }
    }
  }
}
