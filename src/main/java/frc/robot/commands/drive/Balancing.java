package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
    if (Math.abs(gravVec[0]) + Math.abs(gravVec[1]) > 0.05) {
      drive.runVelocity(new ChassisSpeeds(1.6 * gravVec[0], 1.6 * gravVec[1], 0.0));
    } else {
      drive.runVelocity(new ChassisSpeeds());
    }
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
