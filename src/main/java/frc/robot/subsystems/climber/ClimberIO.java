package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  class ClimberIOInputs {
    public double motorPercentOutput = 0.0;
    public double motorVoltage = 0.0;
    public double currentAmpsLeft = 0.0;
    public double appliedVoltsLeft = 0.0;
    public double ClimberLeftPosition = 0.0;
    public double currentAmpsRight = 0.0;
    public double appliedVoltsRight = 0.0;
    public double ClimberRightPosition = 0.0;
    public boolean ratchetLocked = false;
    public boolean reverseLimitSwitchLeftPressed = false;
    public boolean reverseLimitSwitchRightPressed = false;
    public boolean forwardLimitSwitchLeftPressed = false;
    public boolean forwardLimitSwitchRightPressed = false;
  }

  default void updateInputs(ClimberIOInputs inputs) {}

  default void setMotorsOutputPercent(double percentOutput) {}

  default void setRatchetLocked(boolean locked) {}

  default void setLeftMotorPercentOutput(double volts) {}

  default void setRightMotorPercentOutput(double volts) {}

  default boolean getLimitSwitchRight() {
    return true;
  }

  default boolean getLimitSwitchLeft() {
    return true;
  }
  default void resetPosition() {}
}
