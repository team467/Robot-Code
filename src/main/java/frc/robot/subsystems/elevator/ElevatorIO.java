package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  class ElevatorIOInputs {
    // Main motor
    public double velocity;
    public double amps;
    public double volts;
    public double position;
    public double setpoint;
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void setVoltage(double volts) {}
}
