package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
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
    return 0.5;
  } // TODO: tune

  private Translation2d[] moduleTranslations() {
    return new Translation2d[] {
      new Translation2d(Units.inchesToMeters(11.50), Units.inchesToMeters(11.50)),
      new Translation2d(Units.inchesToMeters(11.50), -Units.inchesToMeters(11.50)),
      new Translation2d(-Units.inchesToMeters(11.50), Units.inchesToMeters(11.50)),
      new Translation2d(-Units.inchesToMeters(11.50), -Units.inchesToMeters(11.50))
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

  @Override
  public SimpleFeedforwardConstant moduleDriveFF() {
    return new SimpleFeedforwardConstant(0.15026, 0.13682);
  } // TODO: tune

  @Override
  public SimpleFeedforwardConstant moduleTurnFF() {
    return new SimpleFeedforwardConstant(0.16302, 0.0089689, 0.00034929);
  }

  @Override
  public FeedbackConstant moduleTurnFB() {
    return new FeedbackConstant(3.2526, 0.05);
  }

  @Override
  public SwerveDriveKinematics kinematics() {
    return new SwerveDriveKinematics(moduleTranslations());
  }

  @Override
  public Rotation2d[] absoluteAngleOffset() {
    return new Rotation2d[] {
      Rotation2d.fromDegrees(-46.75781250000001),
      Rotation2d.fromDegrees(11.162109374999998),
      Rotation2d.fromDegrees(110.478515625),
      Rotation2d.fromDegrees(-100.19531250000001)
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

  @Override
  public FeedbackConstant chassisDriveFB() {
    return new FeedbackConstant(0.1);
  } // TODO: tune

  @Override
  public FeedbackConstant chassisTurnFB() {
    return new FeedbackConstant(0.1);
  } // TODO: tune

  @Override
  public int ledChannel() {
    return 2;
  }

  @Override
  public int led2023LedCount() {
    return 10;
  }

  @Override
  public int intakeMotorID() {
    // TODO Update ME
    return 0;
  }

  @Override
  public int intakeCubeLimitSwitchID() {
    // TODO Update Me
    return 0;
  }

  @Override
  public int intakeConeLimitSwitchID() {
    // TODO Update Me
    return 0;
  }
}
