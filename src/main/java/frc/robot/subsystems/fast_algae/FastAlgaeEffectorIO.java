package frc.robot.subsystems.fast_algae;

import org.littletonrobotics.junction.AutoLog;

public interface FastAlgaeEffectorIO {
  @AutoLog
  class FastAlgaeEffectorIOInputs {
    // pivot
    public double pivotVelocity;
    public double pivotAmps;
    public double pivotVolts;
    public double pivotPosition;
    public double pivotMotorTemp;
    public boolean isHighPostion;
    public boolean isLowPostion;
    public boolean isStowed;

  }

  default void updateInputs(FastAlgaeEffectorIOInputs inputs) {}

  default void setPivotVolts(double volts) {
  }

  default void resetPivotPosition(double pivotPosition) {
  }
}
