package frc.robot.subsystems.magicCarpet;

import frc.lib.mechanisms.flywheels.SparkFlywheel;
import frc.robot.Schematic;
import org.wpilib.command3.Command;
import org.wpilib.hardware.bus.CANPort;

public class MagicCarpetSpark implements MagicCarpet {

  SparkFlywheel flywheel;

  public MagicCarpetSpark() {
    flywheel = new SparkFlywheel("Magic Carpet", CANPort.CAN_D0, Schematic.magicCarpetCanId,
        MagicCarpetConstants.MOTOR_INVERTED, MagicCarpetConstants.BELT_VOLTS,
        MagicCarpetConstants.CURRENT_LIMIT);
  }

  @Override
  public Command feed() {
    return flywheel.run();
  }

  @Override
  public Command reverse() {
    return flywheel.reverse();
  }

  @Override
  public Command stop() {
    return flywheel.stop();
  }
}
