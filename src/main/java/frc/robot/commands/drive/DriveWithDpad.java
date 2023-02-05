package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;

public class DriveWithDpad extends CommandBase {
  private Drive drive;
  private Supplier<Integer> povSupplier;

  public DriveWithDpad(Drive drive, Supplier<Integer> povSupplier) {
    this.drive = drive;
    this.povSupplier = povSupplier;
  addRequirements(drive);
  }

  public void execute() {
    int pov = povSupplier.get();

    final double SlowSpeed = 0.3;

    switch (pov) {
      case 0:
        drive.runVelocity(new ChassisSpeeds(SlowSpeed, 0, 0));
      case 45:
        drive.runVelocity(new ChassisSpeeds(SlowSpeed, SlowSpeed, 0));
      case 90:
        drive.runVelocity(new ChassisSpeeds(0, SlowSpeed, 0));
      case 135:
        drive.runVelocity(new ChassisSpeeds(-SlowSpeed, SlowSpeed, 0));
      case 180:
        drive.runVelocity(new ChassisSpeeds(-SlowSpeed, 0, 0));
      case 225:
        drive.runVelocity(new ChassisSpeeds(-SlowSpeed, -SlowSpeed, 0));
      case 270:
        drive.runVelocity(new ChassisSpeeds(0, -SlowSpeed, 0));
      case 315:
        drive.runVelocity(new ChassisSpeeds(SlowSpeed, -SlowSpeed, 0));
      default:
        break;
    }

  }
}
