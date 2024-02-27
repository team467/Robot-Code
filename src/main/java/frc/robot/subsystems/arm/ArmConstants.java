package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
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
  public static final Rotation2d
      HORIZONTAL_OFFSET; // position between our 0 (neutral pick up position) and real 0 (parallel
  // to
  // floor)
  public static final GearRatio GEAR_RATIO;

  static {
    switch (Constants.getRobot()) {
      case ROBOT_2024C -> {
        KG = new TunableNumber("Arm/KG", 0.0);
        KS = new TunableNumber("Arm/KS", 0.0);
        KV = new TunableNumber("Arm/KV", 0.0);
        KP = new TunableNumber("Arm/KP", 25);
        KD = new TunableNumber("Arm/KD", 0.0);
        MAX_VELOCITY = new TunableNumber("Arm/MaxVelocity", 7);
        MAX_ACCELERATION = new TunableNumber("Arm/MaxAcceleration", 12);
        HORIZONTAL_OFFSET = new Rotation2d();
        GEAR_RATIO = new GearRatio(199.73, 1);
      }
      default -> {
        KG = new TunableNumber("Arm/KG", 0.0);
        KS = new TunableNumber("Arm/KS", 0.0);
        KV = new TunableNumber("Arm/KV", 0.0);
        KP = new TunableNumber("Arm/KP", 0.0);
        KD = new TunableNumber("Arm/KD", 0.0);
        MAX_VELOCITY = new TunableNumber("Arm/MaxVelocity", 0.0);
        MAX_ACCELERATION = new TunableNumber("Arm/MaxAcceleration", 0.0);
        HORIZONTAL_OFFSET = new Rotation2d();
        GEAR_RATIO = new GearRatio();
      }
    }
  }
}
