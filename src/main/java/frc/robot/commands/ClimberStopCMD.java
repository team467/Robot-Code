package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber2020;

public class ClimberStopCMD extends CommandBase {
  private final Climber2020 climber;

  public ClimberStopCMD(Climber2020 climber) {
    this.climber = climber;

    addRequirements(climber);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    climber.stop();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
