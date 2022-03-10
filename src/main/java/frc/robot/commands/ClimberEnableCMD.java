package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Climber2020;

public class ClimberEnableCMD extends InstantCommand {
  private final Climber2020 climber;

  public ClimberEnableCMD(Climber2020 climber) {
    this.climber = climber;

    addRequirements(climber);
  }

  @Override
  public void initialize() {
    climber.enable();
  }

  @Override
  public void end(boolean interrupted) {}
}
