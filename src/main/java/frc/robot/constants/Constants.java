package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.constantcontrol.GearRatio;
import frc.lib.constantcontrol.LoggedFeedbackConstant;
import frc.lib.constantcontrol.LoggedSimpleFeedforwardConstant;

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
    return switch (robot()) {
      case ROBOT_COMP, ROBOT_BRIEFCASE -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case ROBOT_SIMBOT -> Mode.SIM;
      default -> Mode.REAL;
    };
  }

  enum RobotType {
    ROBOT_COMP,
    ROBOT_SIMBOT,
    ROBOT_BRIEFCASE
  }

  enum Mode {
    REAL,
    REPLAY,
    SIM
  }

  // Drive constants
  double driveMaxCoastVelocity();

  double maxLinearSpeed();

  double maxAngularSpeed();

  double moduleWheelDiameter();

  GearRatio moduleDriveGearRatio();

  GearRatio moduleTurnGearRatio();

  LoggedSimpleFeedforwardConstant moduleDriveFF();

  LoggedSimpleFeedforwardConstant moduleTurnFF();

  LoggedFeedbackConstant moduleTurnFB();

  SwerveDriveKinematics kinematics();

  Rotation2d[] absoluteAngleOffset();

  double chassisDriveMaxVelocity();

  double chassisDriveMaxAcceleration();

  double chassisTurnMaxVelocity();

  double chassisTurnMaxAcceleration();

  LoggedFeedbackConstant chassisDriveFB();

  LoggedFeedbackConstant chassisTurnFB();
}
