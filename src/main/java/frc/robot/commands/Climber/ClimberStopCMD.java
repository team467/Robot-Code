package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;

public class ClimberStopCMD extends Command {
  private final Climber climber;

  public ClimberStopCMD(Climber climber) {
    this.climber = climber;

    //    addRequirements(climber);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    climber.stop();
  }

  @Override
  public void end(boolean interrupted) {}
  /*
  @Override
  public boolean isFinished() {
    return climber.isStopped();
  }*/
}
