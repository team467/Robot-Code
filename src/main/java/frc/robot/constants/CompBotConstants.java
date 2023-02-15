package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.constantcontrol.GearRatio;
import frc.lib.constantcontrol.LoggedFeedbackConstant;
import frc.lib.constantcontrol.LoggedSimpleFeedforwardConstant;
import java.util.Arrays;

public class CompBotConstants implements Constants {

  @Override
  public RobotType robot() {
    return RobotType.ROBOT_COMP;
  }

  @Override
  public String logFolder() {
    //    return "/media/sda1";
    return null;
  }

  // Drive constants
  @Override
  public double driveMaxCoastVelocity() {
    return 0.5;
  } // TODO: tune

  private Translation2d[] moduleTranslations() {
    return new Translation2d[] {
      new Translation2d(Units.inchesToMeters(12.75), Units.inchesToMeters(9.25)),
      new Translation2d(Units.inchesToMeters(12.75), -Units.inchesToMeters(9.25)),
      new Translation2d(-Units.inchesToMeters(12.75), Units.inchesToMeters(9.25)),
      new Translation2d(-Units.inchesToMeters(12.75), -Units.inchesToMeters(9.25))
    };
  }

  @Override
  public double maxLinearSpeed() {
    return 3.0;
  } // TODO: tune

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
    return Units.inchesToMeters(4);
  }

  @Override
  public GearRatio moduleDriveGearRatio() {
    return new GearRatio(6.75, 1); // SDS L2
  }

  @Override
  public GearRatio moduleTurnGearRatio() {
    return new GearRatio(12.8, 1);
  }

  private static final LoggedSimpleFeedforwardConstant moduleDriveFF =
      new LoggedSimpleFeedforwardConstant(0.15026, 0.13682, "moduleDriveFF");

  @Override
  public LoggedSimpleFeedforwardConstant moduleDriveFF() {
    return moduleDriveFF;
  } // TODO: tune

  private static final LoggedSimpleFeedforwardConstant moduleTurnFF =
      new LoggedSimpleFeedforwardConstant(0.16302, 0.0089689, 0.00034929, "moduleTurnFF");

  @Override
  public LoggedSimpleFeedforwardConstant moduleTurnFF() {
    return moduleTurnFF;
  }

  private static final LoggedFeedbackConstant moduleTurnFB =
      new LoggedFeedbackConstant(3.2526, 0.05, "moduleTurnFB");

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
      Rotation2d.fromDegrees(20.5),
      Rotation2d.fromDegrees(42.0),
      Rotation2d.fromDegrees(168.4),
      Rotation2d.fromDegrees(99.9)
    };
  }

  @Override
  public double chassisDriveMaxVelocity() {
    return 1.2;
  } // TODO: tune

  @Override
  public double chassisDriveMaxAcceleration() {
    return 1.2;
  } // TODO: tune

  @Override
  public double chassisTurnMaxVelocity() {
    return 0.2;
  } // TODO: tune

  @Override
  public double chassisTurnMaxAcceleration() {
    return 0.2;
  } // TODO: tune

  private static LoggedFeedbackConstant chassisDriveFB =
      new LoggedFeedbackConstant(0.1, "chassisDriveFB");

  @Override
  public LoggedFeedbackConstant chassisDriveFB() {
    return chassisDriveFB;
  } // TODO: tune

  private static LoggedFeedbackConstant chassisTurnFB =
      new LoggedFeedbackConstant(0.1, "chassisDriveFB");

  @Override
  public LoggedFeedbackConstant chassisTurnFB() {
    return chassisTurnFB;
  } // TODO: tune
}
