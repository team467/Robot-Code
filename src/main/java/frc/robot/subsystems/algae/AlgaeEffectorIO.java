package frc.robot.subsystems.algae;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeEffectorIO {
  @AutoLog
  class AlgaeEffectorIOInputs {
    // removal
    public double removalVelocity;
    public double removalAmps;
    public double removalVolts;
    // pivot
    public double pivotVelocity;
    public double pivotAmps;
    public double pivotVolts;
    public double pivotPosition;
    public boolean forwardLimitSwitch = false;
    public boolean reverseLimitSwitch = false;
  }

  default void updateInputs(AlgaeEffectorIOInputs inputs) {}
  // TODO: Add in AutoLogged
  default void setRemovalVolts(double volts) {}

  default void setPivotVolts(double volts) {}
}
