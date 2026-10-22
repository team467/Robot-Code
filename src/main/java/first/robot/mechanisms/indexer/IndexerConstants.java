package first.robot.mechanisms.indexer;

import first.robot.Schematic;
import org.wpilib.hardware.bus.CANPort;

public final class IndexerConstants {
  public static final CANPort CAN_PORT = Schematic.canPort;

  public static final boolean FEEDUP_INVERTED = false;
  public static final int FEEDUP_CURRENT_LIMIT = 30;
  public static final double VOLTAGE_COMPENSATION = 12;

  public static final double FEEDUP_VOLT = 12;

  private IndexerConstants() {}
}
