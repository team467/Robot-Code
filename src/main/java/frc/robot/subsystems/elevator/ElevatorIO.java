package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  class ElevatorIOInputs {
    public double positionMeters = 0.0;
    public double velocityMetersPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double setpoint = 0.0;
    public boolean stowLimitSwitch = false;

    public boolean atSetpoint = false;
    public double goalPositionMeters = 0.0;
    public boolean isCalibrated = false;
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void setPosition(double setpoint) {}

  default void setPercent(double percent) {}

  default void goToSetpoint() {}
}
