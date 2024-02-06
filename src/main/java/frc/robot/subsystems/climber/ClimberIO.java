package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  class ClimberIOInputs {

    // public double leftMotorVelocity = 0.0;
    // public double rightMotorVelocity = 0.0;
    // public double leftMotorAppliedVolts = 0.0;
    // public double rightMotorAppliedVolts = 0.0;
    public double leftMotorPercentOutput = 0.0;
    public double rightMotorPercentOutput = 0.0;
    public double leftMotorVoltage = 0.0;
    public double rightMotorVoltage = 0.0;
  }

  default void updateInput(ClimberIOInputs inputs) {}

  // default void setLeftMotorVelocity(double velocity) {}

  // default void setRightMotorVelocity(double velocity) {}

  // default void setLeftMotorCurrent(double current) {}

  // default void setRightMotorCurrent(double current) {}

  // default void setLeftMotorVolts(double volts) {}

  // default void setRightMotorVolts(double volts) {}

  default void setLeftMotorOutputPercent(double percentOutput) {}

  default void setRightMotorOutputPercent(double percentOutput) {}

  default void setLeftMotorInverted() {}

  default void setRightMotorInverted() {}
}
