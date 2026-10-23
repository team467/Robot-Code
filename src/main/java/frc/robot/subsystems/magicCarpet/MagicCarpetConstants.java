package frc.robot.subsystems.magicCarpet;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Volts;

import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

public final class MagicCarpetConstants {

  public static final boolean MOTOR_INVERTED = false;

  public static final Current CURRENT_LIMIT = Amps.of(20);
  public static final Voltage BELT_VOLTS = Volts.of(16);
}
