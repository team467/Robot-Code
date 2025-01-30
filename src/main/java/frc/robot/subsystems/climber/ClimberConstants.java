package frc.robot.subsystems.climber;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ClimberConstants {
  public static final double LOWER_WINCHED_POSITION;
  public static final double UPPER_WINCHED_POSITION;
  public static final double WINCHED_POSITION;
  public static final double LOWER_DEPLOYED_POSITION;
  public static final double UPPER_DEPLOYED_POSITION;
  public static final double DEPLOYED_POSITION;
  public static final int CLIMBER_MOTOR_ID;
  public static final int CLIMBER_LEADER_ID;
  public static final int CLIMBER_FOLLOWER_ID;

  // Climber physics
  public static final int CLIMBER_NUM_MOTORS;
  public static final int CLIMBER_GEAR_RATIO;
  public static final double CLIMBER_GEARING;
  public static final double CARRIAGE_MASS_KG;
  public static final double CLIMBER_DRUM_RADIUS;
  public static final double MIN_CLIMBER_HEIGHT_METERS;
  public static final double MAX_CLIMBER_HEIGHT_METERS;
  public static final boolean SIMULATE_GRAVITY;
  public static final double STARTING_HEIGHT_METERS;
  public static final double MEASUREMENT_STD_DEVS;
  public static final double NEO_PULSES_PER_REVOLUTION;
  public static final double CLIMBER_CONVERSION_FACTOR;

  static {
    switch (Constants.getRobot()) {
      case ROBOT_2025_COMP -> {
        // Real variables
        LOWER_WINCHED_POSITION = 0.35;
        UPPER_WINCHED_POSITION = 0.45;
        WINCHED_POSITION = 0.4;
        LOWER_DEPLOYED_POSITION = 0.95;
        UPPER_DEPLOYED_POSITION = 1.1;
        DEPLOYED_POSITION = 1.0;
        CLIMBER_LEADER_ID = 1;
        CLIMBER_FOLLOWER_ID = 2;

        // Simulated Variables
        CLIMBER_MOTOR_ID = 0;
        CLIMBER_NUM_MOTORS = 2;
        CLIMBER_GEAR_RATIO = 10;
        CLIMBER_GEARING = 10.0;
        CARRIAGE_MASS_KG = 4.0;
        CLIMBER_DRUM_RADIUS = Units.inchesToMeters(2.0);
        MIN_CLIMBER_HEIGHT_METERS = 0.0;
        MAX_CLIMBER_HEIGHT_METERS = 1.0;
        SIMULATE_GRAVITY = true;
        STARTING_HEIGHT_METERS = 0.0;
        MEASUREMENT_STD_DEVS = 0.01;
        NEO_PULSES_PER_REVOLUTION = 4096;
        CLIMBER_CONVERSION_FACTOR = 2.0 * Math.PI * CLIMBER_DRUM_RADIUS / NEO_PULSES_PER_REVOLUTION;
      }

      case ROBOT_SIMBOT -> {
        // Real variables
        LOWER_WINCHED_POSITION = 0.35;
        UPPER_WINCHED_POSITION = 0.45;
        WINCHED_POSITION = 0.4;
        LOWER_DEPLOYED_POSITION = 0.95;
        UPPER_DEPLOYED_POSITION = 1.1;
        DEPLOYED_POSITION = 1.0;
        CLIMBER_LEADER_ID = 1;
        CLIMBER_FOLLOWER_ID = 2;

        // Simulated Variables
        CLIMBER_MOTOR_ID = 11;
        CLIMBER_NUM_MOTORS = 2;
        CLIMBER_GEAR_RATIO = 10;
        CLIMBER_GEARING = 10.0;
        CARRIAGE_MASS_KG = 4.0;
        CLIMBER_DRUM_RADIUS = Units.inchesToMeters(2.0);
        MIN_CLIMBER_HEIGHT_METERS = 0.0;
        MAX_CLIMBER_HEIGHT_METERS = 1.0;
        SIMULATE_GRAVITY = true;
        STARTING_HEIGHT_METERS = 0.0;
        MEASUREMENT_STD_DEVS = 0.01;
        NEO_PULSES_PER_REVOLUTION = 4096;
        CLIMBER_CONVERSION_FACTOR = 2.0 * Math.PI * CLIMBER_DRUM_RADIUS / NEO_PULSES_PER_REVOLUTION;
      }

      default -> {
        // Real variables
        LOWER_WINCHED_POSITION = 0.0;
        UPPER_WINCHED_POSITION = 0.0;
        WINCHED_POSITION = 0.0;
        LOWER_DEPLOYED_POSITION = 0.0;
        UPPER_DEPLOYED_POSITION = 0.0;
        DEPLOYED_POSITION = 0.0;
        CLIMBER_LEADER_ID = 0;
        CLIMBER_FOLLOWER_ID = 0;

        // Simulated Variables
        CLIMBER_MOTOR_ID = 0;
        CLIMBER_NUM_MOTORS = 0;
        CLIMBER_GEAR_RATIO = 0;
        CLIMBER_GEARING = 0.0;
        CARRIAGE_MASS_KG = 0.0;
        CLIMBER_DRUM_RADIUS = 0.0;
        MIN_CLIMBER_HEIGHT_METERS = 0.0;
        MAX_CLIMBER_HEIGHT_METERS = 0.0;
        SIMULATE_GRAVITY = false;
        STARTING_HEIGHT_METERS = 0.0;
        MEASUREMENT_STD_DEVS = 0.0;
        NEO_PULSES_PER_REVOLUTION = 0.0;
        CLIMBER_CONVERSION_FACTOR = 0.0;
      }
    }
  }
}
