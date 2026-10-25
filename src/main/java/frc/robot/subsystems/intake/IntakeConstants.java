package frc.robot.subsystems.intake;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Volts;

import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

public class IntakeConstants {

  public static final double EXTEND_POSITION_CONVERSION = 1;
  public static final double EXTEND_VELOCITY_CONVERSION = 1;

  public static final double STALL_VELOCITY = 0.1;
  public static final Voltage INTAKE_VOLTS = Volts.of(8);

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
  public static final double SLOW_VOLTS = 2;

  public static final double INTAKE_EXTEND_MOTOR_CURRENT_LIMIT = 30;
  public static final Current INTAKE_ROLLERS_MOTOR_CURRENT_LIMIT = Amps.of(45);
}
