package frc.robot.subsystems.climber;

import frc.robot.Constants;

public class ClimberConstants {
  public static final double CLIMBER_CLIMB_VOLTAGE;
  public static final double CLIMBER_HOLD_VOLTAGE;
  public static final double CLIMBER_RELEASE_VOLTAGE;
  public static final int CLIMBER_RATCHET_ID;

  static {
    switch (Constants.getRobot()) {
      case ROBOT_2024C -> {
        CLIMBER_CLIMB_VOLTAGE = 0.0;
        CLIMBER_HOLD_VOLTAGE = 0.0;
        CLIMBER_RELEASE_VOLTAGE = 0.0;
        CLIMBER_RATCHET_ID = 0;
      }
      default -> {
        CLIMBER_CLIMB_VOLTAGE = 0.0;
        CLIMBER_HOLD_VOLTAGE = 0.0;
        CLIMBER_RELEASE_VOLTAGE = 0.0;
        CLIMBER_RATCHET_ID = 0;
      }
    }
  }
}
