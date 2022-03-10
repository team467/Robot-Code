package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter2020;

public class ShooterStopFlywheelCMD extends CommandBase {
  private final Shooter2020 shooter;

  public ShooterStopFlywheelCMD(Shooter2020 shooter) {
    this.shooter = shooter;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    shooter.stopFlywheel();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
