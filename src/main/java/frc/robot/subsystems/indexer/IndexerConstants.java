package frc.robot.subsystems.indexer;

import static org.wpilib.units.Units.Volts;

import frc.lib.utils.MakeTunable;
import org.wpilib.units.measure.Voltage;

@MakeTunable
public class IndexerConstants {

  public static Voltage FEEDUP_VOLT = Volts.of(12);
}
