package frc.robot.subsystems.fastalgae;

import org.littletonrobotics.junction.AutoLog;

public interface FastAlgaeEffectorIO {
  @AutoLog
  class FastAlgaeEffectorIOInputs {
    public double pivotVelocity;
    public double pivotAmps;
    public double pivotVolts;
    public double pivotPosition;
    public double pivotMotorTemp;

    // positions
    public boolean isHighPostion;
    public boolean isLowPostion;
    public boolean isStowed;
  }

  default void updateInputs(FastAlgaeEffectorIOInputsAutoLogged inputs) {}

  default void setPivotVolts(double volts) {}

  default void resetPivotPosition(double pivotPosition) {}
}
