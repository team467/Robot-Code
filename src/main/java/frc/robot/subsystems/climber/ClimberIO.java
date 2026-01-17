package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  class ClimberIOInputs {
    public double volts = 0.0;
  }

  default void updateInputs(ClimberIOInputs inputs) {}

  default void setVoltage(double volts) {}
}
