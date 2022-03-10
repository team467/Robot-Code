package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber2020;

public class ClimberDownCMD extends CommandBase {
  private Climber2020 climber;

  public ClimberDownCMD(Climber2020 climber) {
    this.climber = climber;

    addRequirements(climber);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    climber.down();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
