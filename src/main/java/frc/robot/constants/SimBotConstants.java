package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.constantcontrol.GearRatio;
import frc.lib.constantcontrol.LoggedFeedbackConstant;
import frc.lib.constantcontrol.LoggedSimpleFeedforwardConstant;
import java.util.Arrays;

public class SimBotConstants implements Constants {

  @Override
  public RobotType robot() {
    return RobotType.ROBOT_COMP;
  }

  @Override
  public String logFolder() {
    return "";
  }

  // Drive constants
  @Override
  public double driveMaxCoastVelocity() {
    return 0;
  }

  private Translation2d[] moduleTranslations() {
    return new Translation2d[] {
      new Translation2d(0.65 / 2, 0.65 / 2),
      new Translation2d(0.65 / 2, -0.65 / 2),
      new Translation2d(-0.65 / 2, 0.65 / 2),
      new Translation2d(-0.65 / 2, -0.65 / 2)
    };
  }

  @Override
  public double maxLinearSpeed() {
    return Units.feetToMeters(14.5);
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
    return Units.inchesToMeters(2);
  }

  @Override
  public GearRatio moduleDriveGearRatio() {
    return new GearRatio(6.75, 1); // SDS L2
  }

  @Override
  public GearRatio moduleTurnGearRatio() {
    return new GearRatio(12.8, 1);
  }

  @Override
  public LoggedSimpleFeedforwardConstant moduleDriveFF() {
    return new LoggedSimpleFeedforwardConstant(0.116970, 0.133240, "moduleDriveFF");
  }

  @Override
  public LoggedSimpleFeedforwardConstant moduleTurnFF() {
    return new LoggedSimpleFeedforwardConstant(0, 0, "moduleTurnFF");
  }

  @Override
  public LoggedFeedbackConstant moduleTurnFB() {
    return new LoggedFeedbackConstant(23.0, 0.0, "moduleTurnFB");
  }

  @Override
  public SwerveDriveKinematics kinematics() {
    return new SwerveDriveKinematics(moduleTranslations());
  }

  @Override
  public Rotation2d[] absoluteAngleOffset() {
    return new Rotation2d[] {new Rotation2d()};
  }

  @Override
  public double chassisDriveMaxVelocity() {
    return Units.inchesToMeters(150.0);
  }

  @Override
  public double chassisDriveMaxAcceleration() {
    return Units.inchesToMeters(200);
  }

  @Override
  public double chassisTurnMaxVelocity() {
    return Units.inchesToMeters(150.0);
  }

  @Override
  public double chassisTurnMaxAcceleration() {
    return Units.inchesToMeters(200);
  }

  @Override
  public LoggedFeedbackConstant chassisDriveFB() {
    return new LoggedFeedbackConstant(0, 0, "chassisDriveFB");
  }

  @Override
  public LoggedFeedbackConstant chassisTurnFB() {
    return new LoggedFeedbackConstant(0, 0, "chassisTurnFB");
  }
}
