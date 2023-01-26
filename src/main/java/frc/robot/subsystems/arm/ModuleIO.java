package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {

  @AutoLog
  class ModuleIOInputs {
    public double extendPosition = 0.0;
    public double extendVelocity = 0.0;
    public double extendAppliedVolts = 0.0;
    public double extendCurrent = 0.0;
    public double extendTemp = 0.0;

    public double rotatePositionAbsolute = 0.0;
    public double rotatePosition = 0.0;
    public double rotateVelocity = 0.0;
    public double rotateAppliedVolts = 0.0;
    public double rotateCurrent = 0.0;
    public double rotateTemp = 0.0;
  }

  default void updateInputs(ModuleIOInputs inputs) {}

  default void setExtendVoltage(double volts) {}

  default void setRotateVoltage(double volts) {}

  default void setExtendBrakeMode(boolean brake) {}

  default void setRotateBrakeMode(boolean brake) {}
}
