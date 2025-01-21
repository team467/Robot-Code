package frc.robot.subsystems.coral;

import frc.lib.utils.TunableNumber;
import frc.robot.Constants;

public class CoralEffectorConstants {
  public static TunableNumber CORAL_EFFECTOR_SPEED_OUT;
  public static TunableNumber CORAL_INTAKE_SPEED;
  public static TunableNumber EFFECTOR_HOLD_PERCENT;
  public static int effectorCurrentMotorLimit = 0;
  public static int effectorEncoderPositionFactor = 0;

  static {
    switch (Constants.getRobot()) {
      case ROBOT_2025_COMP -> {
        CORAL_EFFECTOR_SPEED_OUT = new TunableNumber("CoralEffector/SpeedOut", 0.5);
        CORAL_INTAKE_SPEED = new TunableNumber("CoralEffector/SpeedIn", -0.5);
        EFFECTOR_HOLD_PERCENT = new TunableNumber("CoralEffector/HoldPercent", 0.1);
        effectorCurrentMotorLimit = 35;
        effectorEncoderPositionFactor = 1;
      }

      case ROBOT_SIMBOT -> {
        CORAL_EFFECTOR_SPEED_OUT = new TunableNumber("CoralEffector/SpeedOut", 0.5);
        CORAL_INTAKE_SPEED = new TunableNumber("CoralEffector/SpeedIn", -0.5);
        EFFECTOR_HOLD_PERCENT = new TunableNumber("CoralEffector/HoldPercent", 0.1);
        effectorCurrentMotorLimit = 35;
        effectorEncoderPositionFactor = 1;
      }

      default -> {
        CORAL_EFFECTOR_SPEED_OUT = new TunableNumber("CoralEffector/SpeedOut", 0);
        CORAL_INTAKE_SPEED = new TunableNumber("CoralEffector/SpeedIn", 0);
        EFFECTOR_HOLD_PERCENT = new TunableNumber("CoralEffector/HoldPercent", 0);
        effectorCurrentMotorLimit = 0;
        effectorEncoderPositionFactor = 0;
      }
    }
  }
}
