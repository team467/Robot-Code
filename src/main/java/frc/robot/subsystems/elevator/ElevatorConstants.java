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
  public static final double DUCK_POSITION;
  public static final double INTAKE_POSITION;
  public static final double TOLERANCE;
  public static final int elevatorCanId = 0;
  public static final int elevatorCurrentLimit = 5;

  static {
    switch (Constants.getRobot()) {
      case ROBOT_2025_COMP -> {
        KG = new TunableNumber("Elevator/KG", 0.0);
        KS = new TunableNumber("Elevator/KS", 0.0);
        KV = new TunableNumber("Elevator/KV", 0.0);
        KP = new TunableNumber("Elevator/KP", 0.0);
        KD = new TunableNumber("Elevator/KD", 0.0);
        MAX_VELOCITY = new TunableNumber("Elevator/MaxVelocity", Double.POSITIVE_INFINITY);
        MAX_ACCELERATION = new TunableNumber("Elevator/MaxAcceleration");
        STOW = 0.0;
        TOLERANCE = Units.degreesToRadians(0.25);
        DUCK_POSITION = 0.0;
        INTAKE_POSITION = 0.0;
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
        DUCK_POSITION = 0.0;
        INTAKE_POSITION = 0.0;
      }
    }
  }
}
