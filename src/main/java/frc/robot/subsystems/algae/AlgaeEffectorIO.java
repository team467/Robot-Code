package frc.robot.subsystems.algae;

public interface AlgaeEffectorIO {
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
  }

  default void updateInputs(AlgaeEffectorIOInputs inputs) {}
  // TODO: Add in AutoLogged
  default void setRemovalVolts(double volts) {}

  default void setPivotVolts(double volts) {}
}
