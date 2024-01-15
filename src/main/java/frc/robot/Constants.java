package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.Map;

public class Constants {
  private static final RobotType robot = RobotType.ROBOT_2023;
  public static final Measure<Time> loopPeriod = Milliseconds.of(20);
  public static final boolean tuningMode = false;

  public static RobotType getRobot() {
    if (RobotBase.isReal()) {
      if (robot == RobotType.ROBOT_SIMBOT) { // Invalid robot
        DriverStation.reportWarning(
            "Invalid robot selected, using competition robot as default.", false);
        return RobotType.ROBOT_2023;
      } else {
        return robot;
      }
    } else {
      return robot;
    }
  }

  public static Mode getMode() {
    switch (getRobot()) {
      case ROBOT_2023, ROBOT_2024A, ROBOT_BRIEFCASE -> {
        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      }
      case ROBOT_SIMBOT -> {
        return Mode.SIM;
      }
      default -> {
        return Mode.REAL;
      }
    }
  }

  public static final Map<RobotType, String> logFolders =
      Map.of(RobotType.ROBOT_2023, "/media/sda1");

  public enum RobotType {
    ROBOT_2023,
    ROBOT_2024A,
    ROBOT_BRIEFCASE,
    ROBOT_SIMBOT
  }

  public enum Mode {
    REAL,
    REPLAY,
    SIM
  }

  /** Checks whether the robot the correct robot is selected when deploying. */
  public static void main(String... args) {
    if (robot == RobotType.ROBOT_SIMBOT) {
      System.err.println("Cannot deploy, invalid robot selected: " + robot.toString());
      System.exit(1);
    }
  }
}
