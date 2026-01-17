package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  class ClimberIOInputs {
    public double positionDegrees = 0.0;
    public double velocityDegreesPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double targetRotation = 0.0;
    public boolean atTargetRotation = false;
    public boolean limitSwitch = false;
    public boolean isCalibrated = false;
  }

  default void updateInputs(ClimberIOInputs inputs) {}

  default void setPercent(double percent) {}

  default void setRotation(double degrees) {}

  default void goToRotation() {}
}
