package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

public class DriveWithDpad extends Command {
  private final Drive drive;
  private final Supplier<Integer> povSupplier;
  public static final double SLOW_SPEED = Units.inchesToMeters(20);

  public DriveWithDpad(Drive drive, Supplier<Integer> povSupplier) {
    this.drive = drive;
    this.povSupplier = povSupplier;
    addRequirements(drive);
  }

  @Override
  public void execute() {
    int pov = povSupplier.get();

    switch (pov) {
      case 0 -> drive.runVelocity(new ChassisSpeeds(-SLOW_SPEED, 0, 0));
      case 90 -> drive.runVelocity(new ChassisSpeeds(0, SLOW_SPEED, 0));
      case 180 -> drive.runVelocity(new ChassisSpeeds(SLOW_SPEED, 0, 0));
      case 270 -> drive.runVelocity(new ChassisSpeeds(0, -SLOW_SPEED, 0));
      default -> drive.stop();
    }
  }
}
