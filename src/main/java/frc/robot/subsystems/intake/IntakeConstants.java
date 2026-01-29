package frc.robot.subsystems.intake;

public class IntakeConstants {

  public static final double INTAKE_POSITION_CONVERSION = 1;
  public static final double INTAKE_VELOCITY_CONVERSION = 1;

  public static final double EXTEND_POSITION_CONVERSION = 1;
  public static final double EXTEND_VELOCITY_CONVERSION = 1;

  public static final double INTAKE_VOLTS = 3;
  public static final double OUTTAKE_VOLTS = -3;
  public static final double EXTEND_VOLTS = 3;
  public static final double COLLAPSE_VOLTS = -3;

  public static final int INTAKE_MOTOR_ID = 0;
  public static final int INTAKE_EXTEND_ID = 1;
  public static final int EXTEND_LIMIT_ID = 3;

  public static final double PID_P = 0.00005; // arbritrary values
  public static final double PID_I = 0.0000001;
  public static final double PID_D = 0.0000001;

  public static final double EXTEND_POS = 10.0; // TODO: change the actual value
  public static final double COLLAPSE_POS = 0.0;
  public static final double POSITION_TOLERANCE = 0.5;
}
