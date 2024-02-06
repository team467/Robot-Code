package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  class ClimberIOInputs {
    public double motorPercentOutput = 0.0;
    public double motorVoltage = 0.0;
  }

  default void updateInput(ClimberIOInputs inputs) {}

  default void setMotorOutputPercent(double percentOutput) {}


 
}
