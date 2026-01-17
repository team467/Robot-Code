package frc.robot.subsystems.climber;

public class ClimberConstants {
  public static final int CLIMBER_CURRENT_LIMIT = 40;
  public static final int CLIMBER_MOTOR_ID = 0;

  public static final double ENCODER_CONVERSION_FACTOR = 360.0;

  public static final double STARTING_DEGREES = 0.0;

  // PID constants
  public static final double CLIMBER_KP = 1.0;
  public static final double CLIMBER_KI = 0.0;
  public static final double CLIMBER_KD = 0.0;

  public static final double TOLERANCE = 1.0; // measured in degrees

  // Calibration constants
  public static final int LIMIT_SWITCH_ID = 0;
  public static final double CALIBRATION_POSITION_DEGREES =
      0.0; // position when limit switch is hit
  public static final double CALIBRATION_PERCENT = -0.15; // slow speed to find limit switch
}
