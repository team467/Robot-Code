package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DrivetrainStopCMD extends CommandBase {
  private final Drivetrain drivetrain;

  public DrivetrainStopCMD(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    drivetrain.stop();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
