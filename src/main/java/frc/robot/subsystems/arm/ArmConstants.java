package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.utils.TunableNumber;
import frc.robot.Constants;

public class ArmConstants {
  public static final TunableNumber KG;
  public static final TunableNumber KS;
  public static final TunableNumber KV;
  public static final TunableNumber KP;
  public static final TunableNumber KD;
  public static final TunableNumber MAX_VELOCITY;
  public static final TunableNumber MAX_ACCELERATION;
  public static final Rotation2d
      horizontalOffset; // position between our 0 (neutral pick up position) and real 0 (parallel to
  // floor)

  static {
    switch (Constants.getRobot()) {
      case ROBOT_2024A -> {
        KG = new TunableNumber("Arm/KG", 0.0);
        KS = new TunableNumber("Arm/KS", 0.0);
        KV = new TunableNumber("Arm/KV", 0.0);
        KP = new TunableNumber("Arm/KP", 1.0);
        KD = new TunableNumber("Arm/KD", 0.0);
        MAX_VELOCITY = new TunableNumber("Arm/MaxVelocity", Math.PI);
        MAX_ACCELERATION = new TunableNumber("Arm/MaxAcceleration", Math.PI * Math.PI);
        horizontalOffset = new Rotation2d();
      }
      default -> {
        KG = new TunableNumber("Arm/KG", 0.0);
        KS = new TunableNumber("Arm/KS", 0.0);
        KV = new TunableNumber("Arm/KV", 0.0);
        KP = new TunableNumber("Arm/KP", 0.0);
        KD = new TunableNumber("Arm/KD", 0.0);
        MAX_VELOCITY = new TunableNumber("Arm/MaxVelocity", 0.0);
        MAX_ACCELERATION = new TunableNumber("Arm/MaxAcceleration", 0.0);
        horizontalOffset = new Rotation2d();
      }
    }
  }
}
