package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;

public class ClimberLowerCMD extends Command {
  private final Climber climber;

  public ClimberLowerCMD(Climber climber) {
    this.climber = climber;
    // addRequirements(climber);
  }

  @Override
  public void execute() {
    climber.lower();
  }
}
