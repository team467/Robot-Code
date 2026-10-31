package frc.robot.subsystems.intake.extend;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;

public interface IntakeExtend extends Mechanism {

  Command extend();

  Command retract();
}
