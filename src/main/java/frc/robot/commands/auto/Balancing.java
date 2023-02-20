package frc.robot.commands.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;

public class Balancing extends CommandBase {
  private Drive drive;
  private Timer timer = new Timer();
  private double prevZ = 0.0;
  private double deltaZ = 0.0;


  public Balancing(Drive drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    prevZ = drive.getGravVec()[2];
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    double[] gravVec = drive.getGravVec();
    deltaZ = gravVec[2] - prevZ;
    prevZ = gravVec[2];
    double nextZ = gravVec[2] + deltaZ;
//    if (Math.abs(gravVec[2]) < 0.8) {
    if (Math.abs(nextZ) < 0.8) {
      timer.reset();
      drive.runVelocity(new ChassisSpeeds(1.6 * gravVec[0], 1.6 * gravVec[1], 0.0));
    } else {
      drive.runVelocity(new ChassisSpeeds());
    }
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(1.0) && Math.abs(drive.getGravVec()[2]) > 0.8;
  }
}
