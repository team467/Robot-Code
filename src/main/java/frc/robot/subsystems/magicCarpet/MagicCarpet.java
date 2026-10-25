package frc.robot.subsystems.magicCarpet;

import org.wpilib.command3.Command;

public interface MagicCarpet {

  Command feed();

  Command reverse();

  Command stop();
}
