package frc.robot.subsystems.intakerelease;

import frc.robot.subsystems.intakerelease.IntakeRelease.Wants;
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
    public boolean wantsCone = false;
    public boolean wantsCube = false;
  }

  default void updateInputs(IntakeReleaseIOInputs inputs, Wants wants) {}

  default void setVoltage(double volts) {}

  default void setVelocity(double speed) {}
}
