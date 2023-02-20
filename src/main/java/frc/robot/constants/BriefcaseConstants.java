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
      new Translation2d(), new Translation2d(), new Translation2d(), new Translation2d(),
    };
  }

  @Override
  public double maxLinearSpeed() {
    return 0.0;
  }

  @Override
  public double maxAngularSpeed() {
    return (maxLinearSpeed()
        / Arrays.stream(moduleTranslations())
            .map(Translation2d::getNorm)
            .max(Double::compare)
            .get());
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
      new Rotation2d(), new Rotation2d(), new Rotation2d(), new Rotation2d(),
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
  public int armExtendMotorId() {
    return 0;
  }

  @Override
  public int armRotateMotorId() {
    return 0;
  }

  @Override
  public double armExtendConversionFactor() {
    return 1;
  }

  @Override
  public int armRotateHighLimitSwitchId() {
    return 4; // DIO #4
  }

  @Override
  public int armRotateLowLimitSwitchId() {
    return 5; // DIO #5
  }

  @Override
  public int ratchetSolenoidId() {
    return 1;
  }

  @Override
  public double armExtendMaxMeters() {
    return 0.34;
  }

  @Override
  public double armExtendMinMeters() {
    return 0.02;
  }

  @Override
  public double armRotateMaxMeters() {
    return 180;
  }

  @Override
  public double armRotateMinMeters() {
    return 0;
  }

  @Override
  public double armRotateConversionFactor() {
    return 1;
  }

  @Override
  public double armExtendMinDown() {
    // TODO Auto-generated method stub
    return 0;
  }
}
