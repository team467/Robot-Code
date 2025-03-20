package frc.robot.subsystems.fast_algae;

public interface FastAlgaeEffectorIO {

  class FastAlgaeEffectorIOInputs {
    // pivot
    public double pivotVelocity;
    public double pivotAmps;
    public double pivotVolts;
    public double pivotPosition;
    public double pivotMotorTemp;

    // limit switches
    public boolean isStowed = false;
  }

  default void updateInputs(FastAlgaeEffectorIOInputs inputs) {}

  default void setPivotVolts(double volts) {}
}
