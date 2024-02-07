package frc.robot.subsystems.IntakeNote;

import frc.robot.Constants;
import frc.robot.constants.controls.GearRatio;

public class IntakeConstants {
  public static final GearRatio GEAR_RATIO;
  public static final double INTAKE_SPEED;
  public static final double RELEASE_SPEED;
  public static final double STOP_SPEED;

  static {
    switch (Constants.getRobot()) {
      case ROBOT_2024A -> {
        INTAKE_SPEED = 0.2;
        RELEASE_SPEED = -0.2;
        STOP_SPEED = 0.0;
        GEAR_RATIO = new GearRatio(18, 28);
      }

      default -> {
        INTAKE_SPEED = 0.0;
        RELEASE_SPEED = 0.0;
        STOP_SPEED = 0.0;
        GEAR_RATIO = new GearRatio();
      }
    }
  }
}
