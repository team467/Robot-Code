package frc.robot.subsystems.algae;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeEffectorIO {
  @AutoLog
  class AlgaeEffectorIOInputs {
    // removal
    public double removalAmps;
    public double removalVolts;
    public double removalMotorTemp;
    // pivot
    public double pivotVelocity;
    public double pivotAmps;
    public double pivotVolts;
    public double pivotPosition;
    public double pivotMotorTemp;
    // limit switches
    public boolean isFullyExtended = false;
    public boolean isStowed = false;

    public boolean isSpinning = false;
  }

  default void updateInputs(AlgaeEffectorIOInputs inputs) {}

  default void setRemovalVolts(double volts) {}

  default void setPivotVolts(double volts) {}
}
