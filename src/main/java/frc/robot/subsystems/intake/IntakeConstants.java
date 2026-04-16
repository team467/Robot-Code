package frc.robot.subsystems.intake;

public class IntakeConstants {

  public static final double INTAKE_POSITION_CONVERSION = 1;
  public static final double INTAKE_VELOCITY_CONVERSION = 1;

  public static final double EXTEND_POSITION_CONVERSION = 1;
  public static final double EXTEND_VELOCITY_CONVERSION = 1;

  public static final double STALL_TIME = 0.1;
  public static final double STALL_VELOCITY = 0.1;
  public static final double INTAKE_VOLTS = 8;
  public static final double OUTTAKE_VOLTS = -12;
  public static final double EXTEND_VOLTS = 0.01;
  public static final double COLLAPSE_VOLTS = -0.01;
  public static final double INTAKE_MAX_VELOCITY = 200;

  public static final int EXTEND_LIMIT_ID = 0;

  public static final double PID_P = 0.046; // arbritrary values
  public static final double PID_I = 0.000004;
  public static final double PID_D = 0.00001;

  public static final double EXTEND_POS = -13; // TODO: change the actual values
  public static final double FUNNEL_POS = -10;

  // shake around the funnel pos by this much
  public static final double SHAKE_POS_OFFSET = 1;
  public static final double HOME_VOLTAGE = 3;
  public static final double COLLAPSE_POS = 4.0;
  public static final double SLOW_VOLTS = 1.2;
  // How close we need to be to collapse position to stop the intake rollers
  public static final double SAFETY_TOLERANCE = 2;
  public static final double POSITION_TOLERANCE = 7;

  public static final double INTAKE_EXTEND_MOTOR_CURRENT_LIMIT = 30;
  public static final double INTAKE_ROLLERS_MOTOR_CURRENT_LIMIT = 45;
}
