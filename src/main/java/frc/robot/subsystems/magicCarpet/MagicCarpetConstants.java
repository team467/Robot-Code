package frc.robot.subsystems.magicCarpet;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Volts;

import frc.lib.utils.MakeTunable;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

public final class MagicCarpetConstants {

  public static final boolean MOTOR_INVERTED = false;

  @MakeTunable(unit = "Volts")
  public static final Voltage BELT_VOLTS = Volts.of(16);

  @MakeTunable(unit = "Amps")
  public static Current CURRENT_LIMIT = Amps.of(20);
}
