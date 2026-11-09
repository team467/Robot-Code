package frc.robot.subsystems.indexer;

import static org.wpilib.units.Units.Volts;

import frc.lib.utils.Tunable;
import org.wpilib.units.measure.Voltage;

@Tunable
public class IndexerConstants {

  public static Voltage FEEDUP_VOLT = Volts.of(12);
}
