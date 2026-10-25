package frc.robot.subsystems.indexer;

import org.wpilib.command3.Command;

public interface Indexer {

  Command feed();

  Command reverse();

  Command stop();
}
