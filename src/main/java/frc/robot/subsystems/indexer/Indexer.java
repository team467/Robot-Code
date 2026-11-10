package frc.robot.subsystems.indexer;

import org.wpilib.command3.StateMachine;
import org.wpilib.epilogue.Logged;

@Logged
public interface Indexer {

  StateMachine stateMachine();
}
