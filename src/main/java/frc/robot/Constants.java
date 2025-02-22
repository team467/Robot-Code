package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.Map;

public class Constants {
  // Change this temporarily to override the RobotType, e.g. RobotType.ROBOT_SIMBOT
  private static final RobotType ROBOT_TYPE_OVERRIDE = null;
  public static final boolean tuningMode = false;

  private static final String ROBOT_FILENAME = "/home/lvuser/robot";
  private static RobotType cachedRobotTypeFromRoborio = null;

  public static RobotType getRobot() {
    if (cachedRobotTypeFromRoborio == null) {
      cachedRobotTypeFromRoborio = readRobotTypeFromRoborio();
    }
    RobotType robot = cachedRobotTypeFromRoborio;

    if (RobotBase.isReal()) {
      if (robot == RobotType.ROBOT_SIMBOT) { // Invalid robot
        DriverStation.reportWarning(
            "Invalid robot selected, using competition robot as default.", false);
        return RobotType.ROBOT_2025_COMP;
      } else {
        return robot;
      }
    } else {
      return robot;
    }
  }

  private static RobotType readRobotTypeFromRoborio() {
    if (ROBOT_TYPE_OVERRIDE != null) {
      return ROBOT_TYPE_OVERRIDE;
    }
    RobotType robot = null;

    try (FileReader reader = new FileReader(ROBOT_FILENAME)) {
      BufferedReader br = new BufferedReader(reader);
      String robotTypeString = br.readLine();
      robot = RobotType.valueOf(robotTypeString);
      reader.close();
      if (robot == null) {
        throw new RuntimeException(
            "Read invalid RobotType value '"
                + robotTypeString
                + "' from file '"
                + ROBOT_FILENAME
                + "'.");
      }
    } catch (IOException e) {
      throw new RuntimeException("Could not load robot type from file '" + ROBOT_FILENAME + "'.");
    }
    System.out.println("Using RobotType '" + robot + "'");
    cachedRobotTypeFromRoborio = robot;
    return robot;
  }

  public static Mode getMode() {
    switch (getRobot()) {
      case ROBOT_2023, ROBOT_2024_COMP, ROBOT_BRIEFCASE, ROBOT_2025_COMP, ROBOT_2025_TEST -> {
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
      Map.of(
          RobotType.ROBOT_2023, "/media/sda1",
          RobotType.ROBOT_2024_COMP, "/media/sda1");

  public enum RobotType {
    ROBOT_2023,
    ROBOT_2024_COMP,
    ROBOT_2025_COMP,
    ROBOT_2025_TEST,
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
    if (ROBOT_TYPE_OVERRIDE == RobotType.ROBOT_SIMBOT) {
      System.err.println("Cannot deploy, invalid robot selected: " + ROBOT_TYPE_OVERRIDE);
      System.exit(1);
    }
  }
}
