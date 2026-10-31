package frc.robot.subsystems.intake.extend;

import org.wpilib.command3.Mechanism;
import org.wpilib.command3.StateMachine;

public interface IntakeExtend extends Mechanism {

  StateMachine stateMachine();
}
