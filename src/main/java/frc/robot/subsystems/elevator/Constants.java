package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;

public class Constants {

  // Component IDs
  static final int ELEVATOR_MOTOR_ID = 11;

  // PID Controller Constants
  public static final double ELEVATOR_P = 5;
  public static final double ELEVATOR_I = 0.0;
  public static final double ELEVATOR_D = 0.0;

  // Feedforward Constants
  public static final double ELEVATOR_S = 0.0; // volts (V)
  public static final double ELEVATOR_G = 0.762; // volts (V)
  public static final double ELEVATOR_V = 0.762; // volt per velocity (V/(m/s))
  public static final double ELEVATOR_A = 0.0; // volt per acceleration (V/(m/s²))

  // Elevator physics
  static final int ELEVATOR_NUM_MOTORS = 1;
  static final int ELEVATOR_GEAR_RATIO = 1;
  static final double ELEVATOR_GEARING = 10.0;
  static final double CARRIAGE_MASS_KG = 4.0;
  static final double ELEVATOR_DRUM_RADIUS = Units.inchesToMeters(2.0);
  // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
  static final double MIN_ELEVATOR_HEIGHT_METERS = 0.0;
  static final double MAX_ELEVATOR_HEIGHT_METERS = 1.25;
  static final boolean SIMULATE_GRAVITY = true;
  static final double STARTING_HEIGHT_METERS = 0.0;
  static final double MEASUREMENT_STD_DEVS = 0.01;

  // distance per pulse = (distance per revolution) / (pulses per revolution)
  //  = (Pi * D) / ppr
  static final double NEO_PULSES_PER_REVOLUTION = 42;
  static final double ELEVATOR_ENCODER_DISTANCE_PER_PULSE =
      2.0 * Math.PI * ELEVATOR_DRUM_RADIUS / NEO_PULSES_PER_REVOLUTION;
}
