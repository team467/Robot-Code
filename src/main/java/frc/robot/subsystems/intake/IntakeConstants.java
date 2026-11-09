package frc.robot.subsystems.intake;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Volts;

import frc.lib.utils.MakeTunable;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

public class IntakeConstants {

  @MakeTunable
  public static Voltage INTAKE_VOLTS = Volts.of(8);

  public static int EXTEND_LIMIT_ID = 0;

  @MakeTunable
  public static double PID_P = 0.046;

  @MakeTunable
  public static double PID_I = 0.000004;

  @MakeTunable
  public static double PID_D = 0.00001;

  @MakeTunable
  public static double EXTEND_POS = -13;

  // shake around the funnel pos by this much
  @MakeTunable
  public static double HOME_VOLTAGE = 3;

  @MakeTunable
  public static double INTAKE_EXTEND_MOTOR_CURRENT_LIMIT = 30;

  @MakeTunable
  public static Current INTAKE_ROLLERS_MOTOR_CURRENT_LIMIT = Amps.of(45);
}
