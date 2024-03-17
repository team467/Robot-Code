package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.utils.TunableNumber;
import frc.robot.Constants;
import frc.robot.constants.controls.GearRatio;

public class ArmConstants {
  public static final TunableNumber KG;
  public static final TunableNumber KS;
  public static final TunableNumber KV;
  public static final TunableNumber KP;
  public static final TunableNumber KD;
  public static final TunableNumber MAX_VELOCITY;
  public static final TunableNumber MAX_ACCELERATION;
  public static final Rotation2d STOW; // Location of arm when limit switch pressed
  public static final GearRatio GEAR_RATIO;
  public static final Rotation2d AMP_POSITION;
  public static final double TOLERENCE;
  public static final Rotation2d AFTER_INTAKE_POS;

  static {
    switch (Constants.getRobot()) {
      case ROBOT_2024_COMP -> {
        KG = new TunableNumber("Arm/KG", 0);
        KS = new TunableNumber("Arm/KS", 0.0);
        KV = new TunableNumber("Arm/KV", 0.0);
        KP = new TunableNumber("Arm/KP", 66);
        KD = new TunableNumber("Arm/KD", 0);
        MAX_VELOCITY = new TunableNumber("Arm/MaxVelocity", Double.POSITIVE_INFINITY);
        MAX_ACCELERATION = new TunableNumber("Arm/MaxAcceleration", Units.degreesToRadians(900));
        STOW = Rotation2d.fromDegrees(-13.95);
        GEAR_RATIO = new GearRatio(228.571429, 1);
        AMP_POSITION = Rotation2d.fromDegrees(78.26);
        TOLERENCE = Units.degreesToRadians(0.25);
        AFTER_INTAKE_POS = Rotation2d.fromDegrees(-9.75);
      }
      default -> {
        KG = new TunableNumber("Arm/KG", 0.0);
        KS = new TunableNumber("Arm/KS", 0.0);
        KV = new TunableNumber("Arm/KV", 0.0);
        KP = new TunableNumber("Arm/KP", 0.0);
        KD = new TunableNumber("Arm/KD", 0.0);
        MAX_VELOCITY = new TunableNumber("Arm/MaxVelocity", 0.0);
        MAX_ACCELERATION = new TunableNumber("Arm/MaxAcceleration", 0.0);
        STOW = new Rotation2d();
        GEAR_RATIO = new GearRatio();
        AMP_POSITION = Rotation2d.fromRadians(0);
        TOLERENCE = 0;
        AFTER_INTAKE_POS = new Rotation2d();
      }
    }
  }
}
