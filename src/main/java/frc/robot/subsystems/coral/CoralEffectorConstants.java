package frc.robot.subsystems.coral;

import frc.lib.utils.TunableNumber;

public class CoralEffectorConstants {
  public static TunableNumber CORAL_SPEED_OUT = new TunableNumber("CoralEffector/SpeedOut", 0.5);

  public static TunableNumber CORAL_INTAKE_SPEED = new TunableNumber("CoralEffector/SpeedIn", 1);
  public static TunableNumber CORAL_RETAKE_SPEED =
      new TunableNumber("CoralEffector/RetakeSpeed", -0.15);
  public static TunableNumber CORAL_HOLD_PERCENT =
      new TunableNumber("CoralEffector/HoldPercent", 0.1);
  public static TunableNumber EFFECTOR_PULLBACK_SECONDS =
      new TunableNumber("CoralEffector/CurrentLimit", 0.1);
  public static int effectorCurrentMotorLimit = 35;
  public static int effectorEncoderPositionFactor = 1;
}
