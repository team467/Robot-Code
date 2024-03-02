package frc.robot.subsystems.climber;

import frc.lib.utils.TunableNumber;
import frc.robot.Constants;

public class ClimberConstants {
  public static final double CLIMBER_CLIMB_VOLTAGE;
  public static final double CLIMBER_HOLD_VOLTAGE;
  public static final double CLIMBER_RELEASE_VOLTAGE;
  public static final int CLIMBER_RATCHET_ID;
  public static final TunableNumber CLIMBER_KP;
  public static final TunableNumber CLIMBER_KD;
  public static final double ROTS_TO_METERS;

  static {
    switch (Constants.getRobot()) {
      case ROBOT_2024_COMP -> {
        CLIMBER_CLIMB_VOLTAGE = 0.0;
        CLIMBER_HOLD_VOLTAGE = 0.0;
        CLIMBER_RELEASE_VOLTAGE = 0.0;
        CLIMBER_RATCHET_ID = 0;
        CLIMBER_KP = new TunableNumber("Climber/ClimberKP", 0.0);
        CLIMBER_KD = new TunableNumber("Climber/ClimberKD", 0.0);
        ROTS_TO_METERS = 0.0;
      }
      default -> {
        CLIMBER_CLIMB_VOLTAGE = 0.0;
        CLIMBER_HOLD_VOLTAGE = 0.0;
        CLIMBER_RELEASE_VOLTAGE = 0.0;
        CLIMBER_RATCHET_ID = 0;
        CLIMBER_KP = new TunableNumber("Climber/ClimberKP", 0.0);
        CLIMBER_KD = new TunableNumber("Climber/ClimberKD", 0.0);
        ROTS_TO_METERS = 0.0;
      }
    }
  }
}
