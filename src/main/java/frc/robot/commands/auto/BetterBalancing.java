package frc.robot.commands.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class BetterBalancing extends CommandBase {
  private final Drive drive;
  private double angleDegrees;
  private final Timer timer = new Timer();

  public BetterBalancing(Drive drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    angleDegrees = Double.POSITIVE_INFINITY;
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    // Calculate charge station angle and velocity
    angleDegrees =
        drive.getPose().getRotation().getCos() * drive.getPitch().getDegrees()
            + drive.getPose().getRotation().getSin() * drive.getRoll().getDegrees();
    double angleVelocityDegreesPerSec =
        drive.getPose().getRotation().getCos() * Units.radiansToDegrees(drive.getPitchVelocity())
            + drive.getPose().getRotation().getSin()
                * Units.radiansToDegrees(drive.getRollVelocity());
    boolean shouldStop =
        (angleDegrees < 0.0 && angleVelocityDegreesPerSec > 8.0)
            || (angleDegrees > 0.0 && angleVelocityDegreesPerSec < -8.0);

    // Send velocity to drive
    if (shouldStop) {
      drive.stop();
    } else {
      drive.runVelocity(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              Units.inchesToMeters(15.0) * (angleDegrees > 0.0 ? -1.0 : 1.0),
              0.0,
              0.0,
              drive.getPose().getRotation()));
    }

    // Log data
    Logger.getInstance().recordOutput("AutoBalance/AngleDegrees", angleDegrees);
    Logger.getInstance()
        .recordOutput("AutoBalance/AngleVelocityDegreesPerSec", angleVelocityDegreesPerSec);
    Logger.getInstance().recordOutput("AutoBalance/Stopped", shouldStop);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stopWithX();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(angleDegrees) < 3.0 && timer.hasElapsed(2.0);
  }
}
