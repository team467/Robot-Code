package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  class ElevatorIOInputs {
    public double positionMeters = 0.0;
    public double velocityMetersPerSec = 0.0;
    public double elevatorAppliedVolts = 0.0;
    public double[] elevatorCurrentAmps = new double[] {};
    public boolean limitSwitchPressed = false;
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void setVoltage(double volts) {}

  default void resetPosition() {}
}
