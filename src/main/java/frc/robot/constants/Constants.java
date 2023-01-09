package frc.robot.constants;

import edu.wpi.first.wpilibj.RobotBase;

public interface Constants {

  /**
   * @return Robot type/name
   */
  RobotType robot();

  /**
   * @return Folder to put logs into (nullable)
   */
  String logFolder();

  /**
   * @return Check if robot is real, sim, or replay
   */
  default Mode mode() {
    switch (robot()) {
      case ROBOT_COMP:
        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

      case ROBOT_SIMBOT:
        return Mode.SIM;

      default:
        return Mode.REAL;
    }
  }

  enum RobotType {
    ROBOT_COMP,
    ROBOT_SIMBOT
  }

  enum Mode {
    REAL,
    REPLAY,
    SIM
  }
}
