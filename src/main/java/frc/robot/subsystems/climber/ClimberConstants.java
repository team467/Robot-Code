package frc.robot.subsystems.climber;

import frc.lib.utils.TunableNumber;
import frc.robot.Constants;

public class ClimberConstants {
  public static final TunableNumber CLIMBER_KP;
  public static final TunableNumber CLIMBER_KD;
  public static final double ROTS_TO_METERS;

  static {
    switch (Constants.getRobot()) {
      case ROBOT_2024A -> {
        CLIMBER_KP = new TunableNumber("Climber/ClimberKP", 0.0);
        CLIMBER_KD = new TunableNumber("Climber/ClimberKD", 0.0);
        ROTS_TO_METERS = 0.0;
      }
      case ROBOT_SIMBOT -> {
        CLIMBER_KP = new TunableNumber("Climber/ClimberKP", 0.0);
        CLIMBER_KD = new TunableNumber("Climber/ClimberKD", 0.0);
        ROTS_TO_METERS = 0.0;
      }
      default -> {
        CLIMBER_KP = new TunableNumber("Climber/ClimberKP", 0.0);
        CLIMBER_KD = new TunableNumber("Climber/ClimberKD", 0.0);
        ROTS_TO_METERS = 0.0;
      }
    }
  }
}
