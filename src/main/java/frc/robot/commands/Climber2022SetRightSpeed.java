package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.logging.RobotLogManager;
import frc.robot.subsystems.Climber2022;
import java.util.function.Supplier;
import org.apache.logging.log4j.Logger;

public class Climber2022SetRightSpeed extends CommandBase {
  private static final Logger LOGGER = RobotLogManager.getMainLogger(Climber2022StopCMD.class.getName());

  private final Climber2022 climber;
  private final Supplier<Double> speed;

  public Climber2022SetRightSpeed(Climber2022 climber, Supplier<Double> speed) {
    this.climber = climber;
    this.speed = speed;

    addRequirements(climber);
  }

  @Override
  public void initialize() {
  }


  @Override
  public void execute() {
    climber.setRightSpeed(speed.get());
  }


  @Override
  public void end(boolean interrupted) {
    climber.setRightSpeed(0);
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
