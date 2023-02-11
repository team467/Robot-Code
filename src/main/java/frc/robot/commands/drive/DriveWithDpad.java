package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

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

    final double SlowSpeed = Units.inchesToMeters(10);
    System.out.println(pov);

    switch (pov) {
      case 0:
        drive.runVelocity(new ChassisSpeeds(SlowSpeed, 0, 0));
        break;
      case 90:
        drive.runVelocity(new ChassisSpeeds(0, SlowSpeed, 0));
        break;
      case 180:
        drive.runVelocity(new ChassisSpeeds(-SlowSpeed, 0, 0));
        break;
      case 270:
        drive.runVelocity(new ChassisSpeeds(0, -SlowSpeed, 0));
        break;
      default:
        drive.stop();
    }
  }
}
