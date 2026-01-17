package frc.robot.subsystems.intake;

import frc.robot.Constants;

public class IntakeConstants {

  public static final double ENCODER_POSITION_CONVERSION;
  public static final double ENCODER_VELOCITY_CONVERSION;

  public static final int INTAKE_MOTOR_ID;

  static {
    switch (Constants.getRobot()) {
      case ROBOT_2025_COMP -> {
        ENCODER_POSITION_CONVERSION = 0;
        ENCODER_VELOCITY_CONVERSION = 0;

        INTAKE_MOTOR_ID = 0;
      }
      case ROBOT_BRIEFCASE -> {
        ENCODER_POSITION_CONVERSION = 0;
        ENCODER_VELOCITY_CONVERSION = 0;
        INTAKE_MOTOR_ID = 0;
      }
      default -> {
        ENCODER_POSITION_CONVERSION = 0;
        ENCODER_VELOCITY_CONVERSION = 0;
        INTAKE_MOTOR_ID = 0;
      }
    }
  }
}
