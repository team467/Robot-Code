package frc.robot.subsystems.climber;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;


public class ClimberConstants {
 public static final int CLIMBER_RATCHET_ID;
 public static final double CLIMBER_FORWARD_PERCENT;
 public static final double CLIMBER_BACKWARD_PERCENT;
 public static final double BACKUP_TIME;
 public static final float LOWER_LIMIT_POSITION;
 public static final float UPPER_LIMIT_POSITION;
 public static final int CLIMBER_MOTOR_ID;


 // PID Controller Constants
 public static final double CLIMBER_P;
 public static final double CLIMBER_I;
 public static final double CLIMBER_D;


 // Feedforward Constants
 public static final double CLIMBER_S; // volts (V)
 public static final double CLIMBER_G; // volts (V)
 public static final double CLIMBER_V; // volt per velocity (V/(m/s))
 public static final double CLIMBER_A; // volt per acceleration (V/(m/s²))
  // CLimber physics
 static final int CLIMBER_NUM_MOTORS;
 static final int CLIMBER_GEAR_RATIO;
 static final double CLIMBER_GEARING;
 static final double CARRIAGE_MASS_KG;
 static final double CLIMBER_DRUM_RADIUS;
 // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
 static final double MIN_CLIMBER_HEIGHT_METERS;
 static final double MAX_CLIMBER_HEIGHT_METERS;
 static final boolean SIMULATE_GRAVITY;
 static final double STARTING_HEIGHT_METERS;
 static final double MEASUREMENT_STD_DEVS;
 static final double NEO_PULSES_PER_REVOLUTION; // distance per pulse = (distance per revolution) / (pulses per revolution)
 //  = (Pi * D) / ppr
 static final double CLIMBER_CONVERSION_FACTOR;




 static {
   switch (Constants.getRobot()) {
     case ROBOT_2025_COMP -> {
       // Real variables
       CLIMBER_RATCHET_ID = 0;
       CLIMBER_FORWARD_PERCENT = 0.0;
       CLIMBER_BACKWARD_PERCENT = 0.0;
       BACKUP_TIME = 0.0;
       LOWER_LIMIT_POSITION = 0.0f;
       UPPER_LIMIT_POSITION=0.0f;


       // Simulated Variables
       CLIMBER_MOTOR_ID = 0;
       CLIMBER_P = 0.0;
       CLIMBER_I = 0.0;
       CLIMBER_D = 0.0;
       CLIMBER_S = 0.0;
       CLIMBER_G = 0.0;
       CLIMBER_V = 0.0;
       CLIMBER_A = 0.0;
       CLIMBER_NUM_MOTORS = 0;
       CLIMBER_GEAR_RATIO = 0;
       CLIMBER_GEARING = 0;
       CARRIAGE_MASS_KG = 0;
       CLIMBER_DRUM_RADIUS = 0;
       MIN_CLIMBER_HEIGHT_METERS = 0.0;
       MAX_CLIMBER_HEIGHT_METERS = 0.0;
       SIMULATE_GRAVITY = false;
       STARTING_HEIGHT_METERS = 0.0;
       MEASUREMENT_STD_DEVS = 0.0;
       NEO_PULSES_PER_REVOLUTION = 0;
       CLIMBER_CONVERSION_FACTOR = 0.0;
     }
     case ROBOT_SIMBOT -> {
       // Real variables
       CLIMBER_RATCHET_ID = 0;
       CLIMBER_FORWARD_PERCENT = 0.0;
       CLIMBER_BACKWARD_PERCENT = 0.0;
       BACKUP_TIME = 0.0;
       LOWER_LIMIT_POSITION = 0.0f;
       UPPER_LIMIT_POSITION = 0.0f;


       // Simulated Variables
       CLIMBER_MOTOR_ID = 11;
       CLIMBER_P = 5;
       CLIMBER_I = 0.0;
       CLIMBER_D = 0.0;
       CLIMBER_S = 0.0;
       CLIMBER_G = 0.762;
       CLIMBER_V = 0.762;
       CLIMBER_A = 0.0;
       CLIMBER_NUM_MOTORS = 2;
       CLIMBER_GEAR_RATIO = 1;
       CLIMBER_GEARING = 10.0;
       CARRIAGE_MASS_KG = 4.0;
       CLIMBER_DRUM_RADIUS = Units.inchesToMeters(2.0);
       MIN_CLIMBER_HEIGHT_METERS = 0.0;
       MAX_CLIMBER_HEIGHT_METERS = 1.25;
       SIMULATE_GRAVITY = true;
       STARTING_HEIGHT_METERS = 0.0;
       MEASUREMENT_STD_DEVS = 0.01;
       NEO_PULSES_PER_REVOLUTION = 4096;
       CLIMBER_CONVERSION_FACTOR =
     2.0 * Math.PI * CLIMBER_DRUM_RADIUS / NEO_PULSES_PER_REVOLUTION;
     }
     default -> {
       // Real variables
       CLIMBER_RATCHET_ID = 0;
       CLIMBER_FORWARD_PERCENT = 0.0;
       CLIMBER_BACKWARD_PERCENT = 0.0;
       BACKUP_TIME = 0.0;
       LOWER_LIMIT_POSITION = 0.0f;
       UPPER_LIMIT_POSITION = 0.0f;


       // Simulated Variables
       CLIMBER_MOTOR_ID = 0;
       CLIMBER_P = 0.0;
       CLIMBER_I = 0.0;
       CLIMBER_D = 0.0;
       CLIMBER_S = 0.0;
       CLIMBER_G = 0.0;
       CLIMBER_V = 0.0;
       CLIMBER_A = 0.0;
       CLIMBER_NUM_MOTORS = 0;
       CLIMBER_GEAR_RATIO = 0;
       CLIMBER_GEARING = 0;
       CARRIAGE_MASS_KG = 0;
       CLIMBER_DRUM_RADIUS = 0;
       MIN_CLIMBER_HEIGHT_METERS = 0.0;
       MAX_CLIMBER_HEIGHT_METERS = 0.0;
       SIMULATE_GRAVITY = false;
       STARTING_HEIGHT_METERS = 0.0;
       MEASUREMENT_STD_DEVS = 0.0;
       NEO_PULSES_PER_REVOLUTION = 0;
       CLIMBER_CONVERSION_FACTOR = 0.0;
     }
   }
 }
}

