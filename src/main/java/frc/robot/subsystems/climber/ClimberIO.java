package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  class ClimberIOInputs {
    public double motorPercentOutput = 0.0;
    public double motorVoltage = 0.0;
    public boolean ratchetLocked = false;
    public boolean limitSwitchPressed = false;
  }

  default void updateInputs(ClimberIOInputs inputs) {}

  default void setMotorOutputPercent(double percentOutput) {}

  default void setRatchetLocked(boolean locked) {}
}
