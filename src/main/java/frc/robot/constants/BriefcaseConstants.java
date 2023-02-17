package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.constants.controls.FeedbackConstant;
import frc.robot.constants.controls.GearRatio;
import frc.robot.constants.controls.SimpleFeedforwardConstant;
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

  @Override
  public SimpleFeedforwardConstant moduleDriveFF() {
    return new SimpleFeedforwardConstant(0, 0);
  }

  @Override
  public SimpleFeedforwardConstant moduleTurnFF() {
    return new SimpleFeedforwardConstant(0, 0);
  }

  @Override
  public FeedbackConstant moduleTurnFB() {
    return new FeedbackConstant(0);
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

  @Override
  public FeedbackConstant chassisDriveFB() {
    return new FeedbackConstant(0);
  }

  @Override
  public FeedbackConstant chassisTurnFB() {
    return new FeedbackConstant(0);
  }

  @Override
  public int intakeMotorID() {
    // TODO Auto-generated method stub
    return 1;
  }

  @Override
  public int intakeCubeLimitSwitchID() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public int intakeConeLimitSwitchID() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public int ledChannel() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public boolean hasLed2023() {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public int led2023LedCount() {
    // TODO Auto-generated method stub
    return 0;
  }
}
