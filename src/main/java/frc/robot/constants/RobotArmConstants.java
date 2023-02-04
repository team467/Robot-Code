package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.controls.FeedbackConstant;
import frc.robot.constants.controls.GearRatio;
import frc.robot.constants.controls.SimpleFeedforwardConstant;
import java.util.Arrays;

public class RobotArmConstants implements Constants {

  /*
   * @Override
   * public String name() {
   * return "Robot Arm 2023";
   * }
   */

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
      new Translation2d(-0.65 / 2, -0.65 / 2),
      new Translation2d(-0.65 / 2, 0.65 / 2)
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
  public SimpleFeedforwardConstant moduleDriveFF() {
    return new SimpleFeedforwardConstant(0.116970, 0.133240);
  }

  @Override
  public SimpleFeedforwardConstant moduleTurnFF() {
    return new SimpleFeedforwardConstant(0, 0);
  }

  @Override
  public FeedbackConstant moduleTurnFB() {
    return new FeedbackConstant(23.0, 0.0);
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
  public FeedbackConstant chassisDriveFB() {
    return new FeedbackConstant(0, 0);
  }

  @Override
  public FeedbackConstant chassisTurnFB() {
    return new FeedbackConstant(0, 0);
  }
  // Arm Constants

  @Override
  public int armSolenoidChannel() {
    return 1;
  }

  @Override
  public GearRatio armExtendGearRatio() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public GearRatio armRotateGearRatio() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public int armExtendMotorId() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public int armRotateMotorId() {
    // TODO Auto-generated method stub
    return 20;
  }

  @Override
  public double conversionFactor() {
    return 49.17;
  }

  // @Override
  // public boolean hasClimber2020() {
  //     return false;
  // }

  // @Override
  // public int climber2020MotorId() {
  //     return 11;
  // }

  // @Override
  // public boolean climber2020MotorInverted() {
  //     return false;
  // }

  // @Override
  // public double climber2020UpSpeed() {
  //     return 0.3;
  // }

  // @Override
  // public double climber2020DownSpeed() {
  //     return 0.1;
  // }

  // @Override
  // public boolean hasClimber2022() {
  //     return true;
  // }

  // @Override
  // public int climber2022RightMotorId() {
  //     return 6;
  // }

  // @Override
  // public int climber2022LeftMotorId() {
  //     return 11;
  // }

  // @Override
  // public boolean climber2022LeftMotorInverted() {
  //     return false;
  // }

  // @Override
  // public boolean climber2022RightMotorInverted() {
  //     return true;
  // }

  // @Override
  // public double climber2022UpSpeed() {
  //     return 0.8;
  // }

  // @Override
  // public double climber2022DownSpeed() {
  //     return 0.3;
  // }

  //     @Override
  //     public double climber2022LeftLowerLimit() {
  //         return 20 * Math.PI * climber2022Diameter() *
  // climber2022GearRatio().getRotationsPerInput();
  //     }

  //     @Override
  //     public double climber2022RightLowerLimit() {
  //         return 20 * Math.PI * climber2022Diameter() *
  // climber2022GearRatio().getRotationsPerInput();
  //     }

  //     @Override
  //     public double climber2022LeftDangerLimit() {
  //         return -2 * Math.PI * climber2022Diameter() *
  // climber2022GearRatio().getRotationsPerInput();
  //     }

  //     @Override
  //     public double climber2022RightDangerLimit() {
  //         return -2 * Math.PI * climber2022Diameter() *
  // climber2022GearRatio().getRotationsPerInput();
  //     }

  //     @Override
  //     public double climber2022LeftUpperLimit() {
  //         return 75 * Math.PI * climber2022Diameter() *
  // climber2022GearRatio().getRotationsPerInput();
  //     }

  //     @Override
  //     public double climber2022RightUpperLimit() {
  //         return 75 * Math.PI * climber2022Diameter() *
  // climber2022GearRatio().getRotationsPerInput();
  //     }

  //     @Override
  //     public GearRatio climber2022GearRatio() {
  //         return new GearRatio(13.5, 1);
  //     }

  //     @Override
  //     public double climber2022Diameter() {
  //         return 0.0203;
  //     }

}
