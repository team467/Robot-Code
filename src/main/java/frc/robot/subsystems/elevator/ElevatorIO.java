package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  class ElevatorIOInputs {
    public double positionMeters = 0.0;
    public double velocityMetersPerSec = 0.0;
    public double elevatorAppliedVolts = 0.0;
    public double elevatorCurrentAmps = 0.0;
    public double elevatorSetpoint = 0.0;
    public boolean atSetpoint = false;
    public boolean stowLimitSwitch = false;
    public double goalPositionMeters = 0.0;
    public boolean isCalibrated = false;
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void setVoltage(double volts) {}

  default void setPercent(double percent) {}

  default void setPosition(double position) {}

  default void resetPosition(double positionMeters) {}

  default void hold(double holdPosition) {}
}
