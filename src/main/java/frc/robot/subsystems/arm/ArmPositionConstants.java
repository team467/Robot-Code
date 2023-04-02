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

  public static final ArmPosition MID_CONE = new ArmPosition(0.303, 0.156);
  public static final ArmPosition MID_CUBE = new ArmPosition(0.144, 0.114);
  public static final ArmPosition HIGH_CONE = new ArmPosition(0.610, 0.187);
  public static final ArmPosition HIGH_CUBE = new ArmPosition(0.444, 0.154);
  public static final ArmPosition LOW_BOTH = new ArmPosition(0.088, 0.035);
  public static final ArmPosition HOME = new ArmPosition(0.02, 0.0);
  public static final ArmPosition CONE_FLOOR = new ArmPosition(0.233, 0.035);
  public static final ArmPosition CUBE_FLOOR = new ArmPosition(0.234, 0.040);
  public static final ArmPosition SHELF_CONE = new ArmPosition(0.086, 0.159);
  public static final ArmPosition SHELF_CUBE = new ArmPosition(0.086, 0.153);
  public static final ArmPosition FLOOR_RETRACT =
      new ArmPosition(HOME.extendSetpoint, CONE_FLOOR.rotateSetpoint);
}
