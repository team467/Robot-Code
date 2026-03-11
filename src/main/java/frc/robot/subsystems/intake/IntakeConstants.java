package frc.robot.subsystems.intake;

public class IntakeConstants {

  public static final double INTAKE_POSITION_CONVERSION = 1;
  public static final double INTAKE_VELOCITY_CONVERSION = 1;

  public static final double EXTEND_POSITION_CONVERSION = 1;
  public static final double EXTEND_VELOCITY_CONVERSION = 1;

  public static final double STALL_TIME = 0.1;
  public static final double STALL_VELOCITY = 0.1;
  public static final double INTAKE_VOLTS = 12;
  public static final double OUTTAKE_VOLTS = -3;
  public static final double EXTEND_VOLTS = 0.01;
  public static final double COLLAPSE_VOLTS = -0.01;
  public static final double INTAKE_MAX_VELOCITY = 200;

  public static final int EXTEND_LIMIT_ID = 1;

  public static final double PID_P = 0.023; // arbritrary values
  public static final double PID_I = 0.000004;
  public static final double PID_D = 0.00001;

  public static final double EXTEND_POS = -33; // TODO: change the actual values
  public static final double FUNNEL_POS = -16.5;

  // shake around the funnel pos by this much
  public static final double SHAKE_POS_OFFSET = 1;
  public static final double COLLAPSE_POS = 4.0;
  public static final double POSITION_TOLERANCE = 7;

  public static final double INTAKE_INTAKE_MOTOR_CURRENT_LIMIT = 45;
}
