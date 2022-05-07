package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber2022;
import java.util.function.Supplier;

public class Climber2022SetLeftSpeed extends CommandBase {

  private final Climber2022 climber;
  private final Supplier<Double> speed;

  public Climber2022SetLeftSpeed(Climber2022 climber, Supplier<Double> speed) {
    this.climber = climber;
    this.speed = speed;

    addRequirements(climber);
  }

  @Override
  public void initialize() {
  }


  @Override
  public void execute() {
    climber.setLeftSpeed(speed.get());
  }


  @Override
  public void end(boolean interrupted) {
    climber.setLeftSpeed(0);
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
