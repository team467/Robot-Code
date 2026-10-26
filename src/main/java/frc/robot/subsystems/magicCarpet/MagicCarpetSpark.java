package frc.robot.subsystems.magicCarpet;

import frc.lib.mechanisms.flywheels.SparkFlywheel;
import frc.robot.Schematic;
import org.wpilib.hardware.bus.CANPort;

public class MagicCarpetSpark implements MagicCarpet {

  SparkFlywheel flywheel;

  public MagicCarpetSpark() {
    flywheel = new SparkFlywheel(CANPort.CAN_D0, Schematic.magicCarpetCanId,
        MagicCarpetConstants.MOTOR_INVERTED, MagicCarpetConstants.BELT_VOLTS,
        MagicCarpetConstants.CURRENT_LIMIT);
  }

  @Override
  public void feed() {
    flywheel.run();
  }

  @Override
  public void reverse() {
    flywheel.reverse();
  }

  @Override
  public void stop() {
    flywheel.stop();
  }
}
