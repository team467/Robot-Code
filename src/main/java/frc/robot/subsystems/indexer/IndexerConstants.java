package frc.robot.subsystems.indexer;

import static org.wpilib.units.Units.Volts;

import frc.lib.utils.MakeTunable;
import org.wpilib.units.measure.Voltage;

public class IndexerConstants {

  @MakeTunable
  public static Voltage FEEDUP_VOLT = Volts.of(12);
}
