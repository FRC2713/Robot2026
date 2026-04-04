package frc2713.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.GenericHID;
import frc2713.lib.util.LoggedTunableBoolean;
import frc2713.robot.Robot;
import frc2713.robot.RobotContainer;
import frc2713.robot.oi.DevControls;

import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class ShiftManager {

  private static String autoWinner = "";
  private static String autoWinnerInfo = "?";
  private static String autoWinnerColor = "#000000";
  private static String allianceAbbrev = "";
  private static String allianceColor = "#000000";
  private static String opponentColor = "#000000";
  private static boolean isAuto = true;

  private static LoggedTunableBoolean logAllData =
      new LoggedTunableBoolean("matchData/logAllData", false);

  public static double getTimeLeftInActivationPeriod(double time) {
    if (ourHubActive(time)) {
      return getTimeLeftToScore(time);
    } else {
      return getTimeLeftInShift(time);
    }
  }

  public static double getTimeLeftToScore(double time) {
    if (ShiftManager.isAuto) {
      return time;
    }

    if (autoWinner.equals(allianceAbbrev)) {
      // When we won auto: transition (140->130), shift2 (105->80), shift4+endgame (55->0)
      if (time > 130 && time <= 140) {
        return time - 130;
      } else if (time > 80 && time <= 105) {
        return time - 80;
      } else if (time >= 0 && time <= 55) {
        return time;
      } else {
        return 0;
      }
    } else {
      // When we lost auto: transition+shift1 (140->105), shift3 (80->55), endgame (30->0)
      if (time > 105 && time <= 140) {
        return time - 105;
      } else if (time > 55 && time <= 80) {
        return time - 55;
      } else if (time >= 0 && time <= 30) {
        return time;
      } else {
        return 0;
      }
    }
  }

  public static double getTimeLeftInShift(double time) {
    int shiftTime = 25;
    if (time >= 130) {
      return 0;
    } else if ((30 < time) && (time < 130)) {
      return (time - 30) % shiftTime;
    } else {
      return time;
    }
  }

  public static String currentActiveHub(double time) {
    if (ShiftManager.isAuto) {
      return allianceAbbrev;
    }

    if (time >= 130 || time <= 30) {
      return allianceAbbrev;
    }

    int shift;
    if (time > 30 && time < 130) {
      shift = (int) (4 - Math.floor(4 * ((time - 30) / (130 - 30))));
    } else {
      shift = 0;
    }
    if (shift == 0) {
      return "";
    } else if (shift % 2 == 0) {
      return autoWinner;
    } else {
      if (autoWinner.equals("R")) {
        return "B";
      } else if (autoWinner.equals("B")) {
        return "R";
      } else {
        return "";
      }
    }
  }
  // play the rumble on controllers when the last 2 seconds in a shift


  public static String getCurrentPhase(double time) {
    if (ShiftManager.isAuto) {
      return "Auto";
    }

    if (time >= 130) {
      return "Transition";
    } else if (time >= 105) {
      return "Shift 1";
    } else if (time >= 80) {
      return "Shift 2";
    } else if (time >= 55) {
      return "Shift 3";
    } else if (time >= 30) {
      return "Shift 4";
    } else {
      return "Endgame";
    }
  }

  public static boolean ourHubActive(double time) {
    String currentHub = currentActiveHub(time);
    if (ShiftManager.isAuto) {
      return true;
    }

    return currentHub.equals(allianceAbbrev);
  }

  /** For a lot of driver station data, once we get it once we dont really need to ask again */
  public static void pollDriverStationData() {
    // Poll and store the current alliance
    if (allianceAbbrev.length() == 0 || Robot.isSimulation()) {
      Optional<Alliance> currentAlliance = DriverStation.getAlliance();
      if (currentAlliance.isPresent()) {
        allianceAbbrev = currentAlliance.get() == DriverStation.Alliance.Red ? "R" : "B";
        allianceColor = currentAlliance.get() == DriverStation.Alliance.Red ? "#FF0000" : "#0000FF";
        opponentColor = currentAlliance.get() == DriverStation.Alliance.Red ? "#0000FF" : "#FF0000";
      }
    }

    // Poll and store who won auto
    if (autoWinner.length() == 0 || Robot.isSimulation()) {
      String gameMessage = DriverStation.getGameSpecificMessage();
      if (gameMessage.length() > 0) {
        autoWinner = gameMessage.substring(0, 1);
        autoWinnerColor =
            autoWinner.equals("R") ? "#FF0000" : autoWinner.equals("B") ? "#0000FF" : "#000000";
        autoWinnerInfo = autoWinner.equals(allianceAbbrev) ? "YES" : "NO";
      }
    }

    isAuto = DriverStation.isAutonomous();
  }

  public static void periodic() {
    double matchTime = DriverStation.getMatchTime();
    pollDriverStationData();

    // Helps loop time to reduce logs
    if (logAllData.get()) {
      Logger.recordOutput("matchData/timeLeftInShift", getTimeLeftInShift(matchTime));
      Logger.recordOutput("matchData/timeLeftToScore", getTimeLeftToScore(matchTime));
      Logger.recordOutput("matchData/whoWonAuto", autoWinner);
      Logger.recordOutput(
          "matchData/FirstActive",
          autoWinner.equals("R") ? "0000FF" : autoWinner.equals("B") ? "FF0000" : "000000");
      Logger.recordOutput("matchData/autoWinnerColor", autoWinnerColor);
      Logger.recordOutput("matchData/time", matchTime);
    }

    // Things actively being used in elastic_layout.json
    Logger.recordOutput("matchData/currentMatchPhase", getCurrentPhase(matchTime));
    Logger.recordOutput("matchData/ourHubActive", ourHubActive(matchTime));

    Logger.recordOutput("matchData/autoWinnerInfo", autoWinnerInfo);
    Logger.recordOutput(
        "matchData/currentActivationPeriod", getTimeLeftInActivationPeriod(matchTime));
    Logger.recordOutput(
        "matchData/currentActiveHubColor", ourHubActive(matchTime) ? allianceColor : opponentColor);
  }
}
