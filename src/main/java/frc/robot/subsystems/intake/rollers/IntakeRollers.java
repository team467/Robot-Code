package frc.robot.subsystems.intake.rollers;

import org.wpilib.command3.Command;

public interface IntakeRollers {

  Command feed();

  Command reverse();

  Command stop();
}
