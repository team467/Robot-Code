package frc.robot.commands.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class Balancing extends Command {
  private Drive drive;
  private Timer timer = new Timer();

  public Balancing(Drive drive) {
    this.drive = drive;
    addRequirements(drive);

    timer.reset();
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    double[] gravVec = drive.getGravVec();
    //    if (Math.abs(nextZ) < 0.98) {
    Logger.getInstance().recordOutput("BalancingMag", Math.hypot(gravVec[0], gravVec[1]));
    if (Math.abs(Math.hypot(gravVec[0], gravVec[1])) > 0.1) {
      timer.reset();
      drive.runVelocity(new ChassisSpeeds(1.5 * gravVec[0], 1.5 * gravVec[1], 0.0));
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
    return timer.hasElapsed(8.0)
        && Math.abs(Math.hypot(drive.getGravVec()[0], drive.getGravVec()[1])) < 0.1;
  }
}
