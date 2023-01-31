package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;

public class Balancing extends CommandBase {
  Drive drive;

  public Balancing(Drive drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void execute() {
    double[] gravVec = drive.getGravVec();
    drive.runVelocity(new ChassisSpeeds(gravVec[0], gravVec[1], 0.0));
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
