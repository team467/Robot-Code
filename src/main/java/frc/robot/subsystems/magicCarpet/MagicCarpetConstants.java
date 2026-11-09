package frc.robot.subsystems.magicCarpet;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Volts;

import frc.lib.utils.Tunable;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

@Tunable
public final class MagicCarpetConstants {

  public static boolean MOTOR_INVERTED = false;
  public static Voltage BELT_VOLTS = Volts.of(16);
  public static Current CURRENT_LIMIT = Amps.of(20);
}
