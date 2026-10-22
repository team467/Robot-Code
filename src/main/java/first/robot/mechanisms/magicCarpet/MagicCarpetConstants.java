package first.robot.mechanisms.magicCarpet;

import first.robot.Schematic;
import org.wpilib.hardware.bus.CANPort;

public final class MagicCarpetConstants {
  public static final CANPort CAN_PORT = Schematic.canPort;

  public static final boolean MOTOR_INVERTED = false;

  public static final int CURRENT_LIMIT = 20;
  public static final double BELT_SPEED = 0.8;

  private MagicCarpetConstants() {}
}
