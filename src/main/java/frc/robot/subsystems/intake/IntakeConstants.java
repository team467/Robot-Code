package frc.robot.subsystems.intake;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Volts;

import frc.lib.utils.Tunable;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

@Tunable
public class IntakeConstants {

  public static Voltage INTAKE_VOLTS = Volts.of(8);

  public static int EXTEND_LIMIT_ID = 0;

  public static double PID_P = 0.046;
  public static double PID_I = 0.000004;
  public static double PID_D = 0.00001;

  public static double EXTEND_POS = -13;

  // shake around the funnel pos by this much
  public static double HOME_VOLTAGE = 3;

  public static double INTAKE_EXTEND_MOTOR_CURRENT_LIMIT = 30;
  public static Current INTAKE_ROLLERS_MOTOR_CURRENT_LIMIT = Amps.of(45);
}
