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
  public static final Rotation2d OFFSET; // Location of arm when limit switch pressed
  public static final GearRatio GEAR_RATIO;

  static {
    switch (Constants.getRobot()) {
      case ROBOT_2024C -> {
        KG = new TunableNumber("Arm/KG", 0);
        KS = new TunableNumber("Arm/KS", 0.0);
        KV = new TunableNumber("Arm/KV", 0.0);
        KP = new TunableNumber("Arm/KP", 30);
        KD = new TunableNumber("Arm/KD", 0);
        MAX_VELOCITY = new TunableNumber("Arm/MaxVelocity", Units.degreesToRadians(30));
        MAX_ACCELERATION = new TunableNumber("Arm/MaxAcceleration", Units.degreesToRadians(15));
        OFFSET = Rotation2d.fromDegrees(-13.95);
        GEAR_RATIO = new GearRatio(228.571429, 1);
      }
      default -> {
        KG = new TunableNumber("Arm/KG", 0.0);
        KS = new TunableNumber("Arm/KS", 0.0);
        KV = new TunableNumber("Arm/KV", 0.0);
        KP = new TunableNumber("Arm/KP", 0.0);
        KD = new TunableNumber("Arm/KD", 0.0);
        MAX_VELOCITY = new TunableNumber("Arm/MaxVelocity", 0.0);
        MAX_ACCELERATION = new TunableNumber("Arm/MaxAcceleration", 0.0);
        OFFSET = new Rotation2d();
        GEAR_RATIO = new GearRatio();
      }
    }
  }
}
