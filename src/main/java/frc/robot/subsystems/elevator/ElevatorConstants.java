package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;
import frc.lib.utils.TunableNumber;
import frc.robot.Constants;

public class ElevatorConstants {
  public static final TunableNumber KG;
  public static final TunableNumber KS;
  public static final TunableNumber KV;
  public static final TunableNumber KP;
  public static final TunableNumber KD;
  public static final TunableNumber MAX_VELOCITY;
  public static final TunableNumber MAX_ACCELERATION;
  public static final double STOW;
  public static final double INTAKE_POSITION;
  public static final double TOLERANCE;
  public static final double ENCODER_CONVERSION_FACTOR;
  public static final int elevatorCurrentLimit = 30;
  public static final double maxElevatorExtension = Units.inchesToMeters(22.9);
  public static final double elevatorToGround = Units.inchesToMeters(17 + 6 / 8); // 17 + 6 / 8

  static {
    switch (Constants.getRobot()) {
      case ROBOT_2025_COMP -> {
        KG = new TunableNumber("Elevator/KG", 0.0);
        KS = new TunableNumber("Elevator/KS", 0.0);
        KV = new TunableNumber("Elevator/KV", 0.0);
        KP = new TunableNumber("Elevator/KP", 6);
        KD = new TunableNumber("Elevator/KD", 0.0);
        MAX_VELOCITY = new TunableNumber("Elevator/MaxVelocity", Double.POSITIVE_INFINITY);
        MAX_ACCELERATION = new TunableNumber("Elevator/MaxAcceleration", Double.POSITIVE_INFINITY);
        STOW = 0.0;
        TOLERANCE = Units.degreesToRadians(0.25);
        INTAKE_POSITION = 0.0;
        ENCODER_CONVERSION_FACTOR =
            Units.inchesToMeters((76.0 + 13.0 / 16.0) - (17.0 + 6.0 / 8.0))
                / 49.97; // 9 reduction, 27 teeth per rotation, 5 mm per tooth
      }
      default -> {
        KG = new TunableNumber("Elevator/KG", 0.0);
        KS = new TunableNumber("Elevator/KS", 0.0);
        KV = new TunableNumber("Elevator/KV", 0.0);
        KP = new TunableNumber("Elevator/KP", 0.0);
        KD = new TunableNumber("Elevator/KD", 0.0);
        MAX_VELOCITY = new TunableNumber("Elevator/MaxVelocity", 0.0);
        MAX_ACCELERATION = new TunableNumber("Elevator/MaxAcceleration", 0.0);
        STOW = 0.0;
        TOLERANCE = 0.0;
        INTAKE_POSITION = 0.0;
        ENCODER_CONVERSION_FACTOR = 0.0;
      }
    }
  }
}
