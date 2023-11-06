package frc.robot.subsystems.effector;

import frc.robot.subsystems.effector.Effector.Wants;
import org.littletonrobotics.junction.AutoLog;

public interface EffectorIO {
  @AutoLog
  class EffectorIOInputs {
    public double motorPosition = 0.0;
    public double motorVelocity = 0.0;
    public double motorAppliedVolts = 0.0;
    public double motorCurrent = 0.0;
    public boolean cubeLimitSwitch = false;
    public boolean coneLimitSwitch = false;
    public boolean wantsCone = false;
    public boolean wantsCube = false;
  }

  default void updateInputs(EffectorIOInputs inputs, Wants wants) {}

  default void setVoltage(double volts) {}

  default void setPercent(double percent) {}
}
