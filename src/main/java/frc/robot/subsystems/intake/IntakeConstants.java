package frc.robot.subsystems.intake;

import frc.lib.utils.TunableNumber;
import frc.robot.Constants;
import frc.robot.constants.controls.GearRatio;

public class IntakeConstants {
  public static final GearRatio GEAR_RATIO;
  public static final TunableNumber INTAKE_SPEED;
  public static final double RELEASE_SPEED;
  public static final double STOP_SPEED;

  static {
    switch (Constants.getRobot()) {
      case ROBOT_2024_COMP -> {
        INTAKE_SPEED = new TunableNumber("Intake/IntakeSpeed", 1);
        RELEASE_SPEED = -1;
        STOP_SPEED = 0.0;
        GEAR_RATIO = new GearRatio(18, 28);
      }

      default -> {
        INTAKE_SPEED = new TunableNumber("Intake/IntakeSpeed");
        RELEASE_SPEED = 0.0;
        STOP_SPEED = 0.0;
        GEAR_RATIO = new GearRatio();
      }
    }
  }
}
