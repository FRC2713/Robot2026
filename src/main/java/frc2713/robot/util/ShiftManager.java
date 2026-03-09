package frc2713.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class ShiftManager {
  public static double getTimeLeftInShift() {
    double time = DriverStation.getMatchTime();
    int shiftTime = 25;
    if (time >= 130) {
      return 0;
    } else if ((30 < time) && (time < 130)) {
      return (time - 30) % shiftTime;
    } else {
      return time;
    }
  }

  public static String whoWonAuto() {
    String gameMessage = DriverStation.getGameSpecificMessage();
    String firstInactive;
    if (gameMessage.length() > 0) {
      firstInactive = gameMessage.substring(0, 1);
    } else {
      firstInactive = "";
    }
    return firstInactive;
  }

  public static String currentActiveHub() {
    String firstInactive = whoWonAuto();
    double time = DriverStation.getMatchTime();

    int shift;
    if (time > 30 && time < 130) {
      shift = (int) (4 - Math.floor(4 * ((time - 30) / (130 - 30))));
    } else {
      shift = 0;
    }
    if (shift == 0) {
      return "";
    } else if (shift % 2 == 0) {
      return firstInactive;
    } else {
      if (firstInactive.equals("R")) {
        return "B";
      } else if (firstInactive.equals("B")) {
        return "R";
      } else {
        return "";
      }
    }
  }

  public static String getCurrentPhase() {
    double time = DriverStation.getMatchTime();
    if (time >= 105) {
      return "Phase 1";
    } else if (time >= 80) {
      return "Phase 2";
    } else if (time >= 55) {
      return "Phase 3";
    } else if (time >= 30) {
      return "Phase 4";
    } else {
      return "";
    }
  }

  public static boolean ourHubActive() {
    String currentHub = currentActiveHub();
    Optional<Alliance> currentAlliance = DriverStation.getAlliance();
    if (currentAlliance.isPresent()) {
      if (currentHub.equals("R")) {
        return (currentAlliance.get() == DriverStation.Alliance.Red);
      } else if (currentHub.equals("B")) {
        return (currentAlliance.get() == DriverStation.Alliance.Blue);
      }
    }
    return true;
  }

  public static void periodic() {
    Logger.recordOutput("matchData/timeLeftInShift", getTimeLeftInShift());
    Logger.recordOutput("matchData/currentMatchPhase", getCurrentPhase());
    Logger.recordOutput("matchData/ourHubActive", ourHubActive());
    String autoWinner = whoWonAuto();
    Logger.recordOutput("matchData/whoWonAuto", autoWinner);
    Logger.recordOutput(
        "matchData/FirstActive",
        autoWinner.equals("R") ? "0000FF" : autoWinner.equals("B") ? "FF0000" : "000000");
    Logger.recordOutput(
        "matchData/autoWinnerColor",
        "#" + (autoWinner.equals("B") ? "0000FF" : autoWinner.equals("R") ? "FF0000" : "000000"));
    Logger.recordOutput("matchData/time", DriverStation.getMatchTime());
  }
}
