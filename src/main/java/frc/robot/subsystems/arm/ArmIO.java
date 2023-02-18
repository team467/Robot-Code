package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

  @AutoLog
  class ArmIOInputs {
    public double extendPosition = 0.0;
    public double extendVelocity = 0.0;
    public double extendAppliedVolts = 0.0;
    public double extendCurrent = 0.0;
    public double extendTemp = 0.0;

    public double rotatePosition = 0.0;
    public double rotateVelocity = 0.0;
    public double rotateAppliedVolts = 0.0;
    public double rotateCurrent = 0.0;
    public double rotateTemp = 0.0;

    public boolean extendLimitSwitch = false;
    public boolean rotateHighLimitSwitch = false;
    public boolean rotateLowLimitSwitch = false;
  }

  default void updateInputs(ArmIOInputs inputs) {}

  default void setExtendVoltage(double volts) {}

  default void setRotateVoltage(double volts) {}

  default void setExtendVelocity(double speed) {}

  default void setRotateVelocity(double speed) {}

  default void resetEncoderPosition() {}

  default void resetRotateEncoder() {}

  default void setRatchetLocked(boolean locked) {}
}
