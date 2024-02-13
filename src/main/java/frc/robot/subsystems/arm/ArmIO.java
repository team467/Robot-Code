package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  class ArmIOInputs {
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double armAppliedVolts = 0.0;
    public double[] armCurrentAmps = new double[] {};
    public boolean limitSwitchPressed = false;
  }

  default void updateInputs(ArmIOInputs inputs) {}

  default void setVoltage(double volts) {}

  default void resetPosition() {}
}
