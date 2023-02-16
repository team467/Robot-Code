package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.constantcontrol.GearRatio;
import frc.lib.constantcontrol.LoggedFeedbackConstant;
import frc.lib.constantcontrol.LoggedSimpleFeedforwardConstant;
import java.util.Arrays;

public class BriefcaseConstants implements Constants {

  @Override
  public RobotType robot() {
    return RobotType.ROBOT_BRIEFCASE;
  }

  @Override
  public String logFolder() {
    return "/media/sda1";
  }

  // Drive constants
  @Override
  public double driveMaxCoastVelocity() {
    return 0;
  }

  private Translation2d[] moduleTranslations() {
    return new Translation2d[] {
      new Translation2d(), new Translation2d(), new Translation2d(), new Translation2d()
    };
  }

  @Override
  public double maxLinearSpeed() {
    return 0.0;
  }

  @Override
  public double maxAngularSpeed() {
    return maxLinearSpeed()
        / Arrays.stream(moduleTranslations())
            .map(Translation2d::getNorm)
            .max(Double::compare)
            .get();
  }

  @Override
  public double moduleWheelDiameter() {
    return 0;
  }

  @Override
  public GearRatio moduleDriveGearRatio() {
    return new GearRatio();
  }

  @Override
  public GearRatio moduleTurnGearRatio() {
    return new GearRatio();
  }

  private static final LoggedSimpleFeedforwardConstant moduleDriveFF =
      new LoggedSimpleFeedforwardConstant(0, 0, "moduleDriveFF");

  @Override
  public LoggedSimpleFeedforwardConstant moduleDriveFF() {
    return moduleDriveFF;
  }

  private static final LoggedSimpleFeedforwardConstant moduleTurnFF =
      new LoggedSimpleFeedforwardConstant(0, 0, "moduleTurnFF");

  @Override
  public LoggedSimpleFeedforwardConstant moduleTurnFF() {
    return moduleTurnFF;
  }

  private static final LoggedFeedbackConstant moduleTurnFB =
      new LoggedFeedbackConstant(0, "moduleTurnFB");

  @Override
  public LoggedFeedbackConstant moduleTurnFB() {
    return moduleTurnFB;
  }

  @Override
  public SwerveDriveKinematics kinematics() {
    return new SwerveDriveKinematics(moduleTranslations());
  }

  @Override
  public Rotation2d[] absoluteAngleOffset() {
    return new Rotation2d[] {
      new Rotation2d(), new Rotation2d(), new Rotation2d(), new Rotation2d()
    };
  }

  @Override
  public double chassisDriveMaxVelocity() {
    return 0;
  }

  @Override
  public double chassisDriveMaxAcceleration() {
    return 0;
  }

  @Override
  public double chassisTurnMaxVelocity() {
    return 0;
  }

  @Override
  public double chassisTurnMaxAcceleration() {
    return 0;
  }

  private static final LoggedFeedbackConstant chassisDriveFB =
      new LoggedFeedbackConstant(0, "chassisDriveFB");

  @Override
  public LoggedFeedbackConstant chassisDriveFB() {
    return chassisDriveFB;
  }

  private static final LoggedFeedbackConstant chassisTurnFB =
      new LoggedFeedbackConstant(0, "chassisTurnFB");

  @Override
  public LoggedFeedbackConstant chassisTurnFB() {
    return chassisTurnFB;
  }
}
