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

  @Override
  public Translation2d[] moduleTranslations() {
    return new Translation2d[] {
      new Translation2d(), new Translation2d(), new Translation2d(), new Translation2d(),
    };
  }

  @Override
  public double maxLinearSpeed() {
    return 0.1;
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
      new Rotation2d(), new Rotation2d(), new Rotation2d(), new Rotation2d(),
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
    return 0;
  }

  @Override
  public int led2023LedCount() {
    return 10;
  }

  @Override
  public int intakeMotorID() {
    // TODO Update me
    return 11;
  }

  @Override
  public int intakeCubeLimitSwitchID() {
    // TODO: Update me
    return 1;
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
    return 0;
  }

  @Override
  public int armRotateHighLimitSwitchId() {
    return 0;
  }

  @Override
  public int armRotateLowLimitSwitchId() {
    return 0;
  }

  @Override
  public int ratchetSolenoidId() {
    return 0;
  }

  @Override
  public double armExtendMaxMeters() {
    return 0;
  }

  @Override
  public double armExtendMinMeters() {
    return 0;
  }

  @Override
  public double armRotateMaxMeters() {
    return 0;
  }

  @Override
  public double armRotateMinMeters() {
    return 0;
  }

  @Override
  public double armRotateConversionFactor() {
    return 0;
  }

  @Override
  public double armExtendMinDown() {
    return 0;
  }
}
