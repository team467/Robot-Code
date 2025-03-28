package frc.robot.subsystems.coral;

import frc.lib.utils.TunableNumber;

public class CoralEffectorConstants {
  public static final TunableNumber CORAL_SPEED_OUT =
      new TunableNumber("CoralEffector/SpeedOut", 0.5);
  public static final TunableNumber L4_CORAL_SPEED_OUT =
      new TunableNumber("CoralEffector/SpeedOutL4", 0.3);
  public static final TunableNumber L1_CORAL_SPEED_OUT =
      new TunableNumber("CoralEffector/SpeedOutL1", 0.2);

  public static final TunableNumber CORAL_INTAKE_SPEED =
      new TunableNumber("CoralEffector/SpeedIn", 1.0);
  public static final TunableNumber CORAL_RETAKE_SPEED =
      new TunableNumber("CoralEffector/RetakeSpeed", -0.4);
  public static final TunableNumber CORAL_HOLD_PERCENT =
      new TunableNumber("CoralEffector/HoldPercent", 0.1);
  public static final TunableNumber EFFECTOR_PULLBACK_SECONDS =
      new TunableNumber("CoralEffector/PullbackTime", 0.1);

  public static final int EFFECTOR_CURRENT_MOTOR_LIMIT = 35;
  public static final int EFFECTOR_ENCODER_POSITION_FACTOR = 1;
}
