package frc.robot.subsystems.arm;

public class ArmPositionConstants {
  public static class ArmPosition {
    public double extendSetpoint;
    public double rotateSetpoint;

    public ArmPosition(double extendSetpoint, double rotateSetpoint) {
      this.extendSetpoint = extendSetpoint;
      this.rotateSetpoint = rotateSetpoint;
    }
  }
  // TODO: UPDATE ME
  public static final ArmPosition MID_CONE = new ArmPosition(0.302, 0.289);
  public static final ArmPosition MID_CUBE = new ArmPosition(0.192, 0.221);
  public static final ArmPosition HIGH_CONE = new ArmPosition(0.682, 0.348);
  public static final ArmPosition HIGH_CUBE = new ArmPosition(0.479, 0.291);
  public static final ArmPosition LOW_BOTH = new ArmPosition(0.164, 0.071);
  public static final ArmPosition HOME = new ArmPosition(0.02, 0.0);
  public static final ArmPosition FLOOR = new ArmPosition(0.22, 0.065);
  public static final ArmPosition SHELF = new ArmPosition(0.086, 0.310);
}
