package first.robot;

import org.wpilib.framework.RobotBase;

public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  private static final RobotType robot = RobotType.ROBOT_2026_COMP;

  public static RobotType getRobot() {
    return robot;
  }

  public static enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public static enum RobotType {
    ROBOT_2025_COMP,
    ROBOT_2026_COMP,
    ROBOT_BRIEFCASE
  }

  private Constants() {}
}
