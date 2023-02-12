package frc.robot.subsystems.intakerelease;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeReleaseIO {
  @AutoLog
  class IntakeReleaseIOInputs {
    public double motorPosition = 0.0;
    public double motorVelocity = 0.0;
    public double motorAppliedVolts = 0.0;
    public double motorCurrent = 0.0;
    public double motorTemp = 0.0;
    public boolean cubeLimitSwitch = false;
    public boolean coneLimitSwitch = false;
  }

  default void updateInputs(IntakeReleaseIOInputs inputs) {}

  default void setVoltage(double volts) {}

  default void setVelocity(double speed) {}
}
