package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

public class Autos {
  private final Drive drive;

  public Autos(Drive drive) {
    this.drive = drive;
  }

  private static final Supplier<Pose2d> climb =
      () -> new Pose2d(1.583, 3.750, new Rotation2d(Units.degreesToRadians(180.000)));
  private static final Supplier<Pose2d> center = () -> new Pose2d(3.504, 4.019, new Rotation2d(0));
  private static final Supplier<Pose2d> CenterA =
      () -> new Pose2d(3.457, 4.941, new Rotation2d(Units.degreesToRadians(-55.305)));

  public Command CenterA() {
    return Commands.sequence(
        Commands.runOnce(() -> drive.setPose(CenterA.get())),
        drive.getAutonomousCommand("CA-out-intake"),
        new StraightDriveToPose(climb.get(), drive).withTimeout(2.0));
  }

  public Command testPath() {
    return Commands.sequence(
        Commands.runOnce(
            () ->
                drive.setPose(new Pose2d(3.213, 5.600, new Rotation2d(Units.degreesToRadians(0))))),
        drive.getAutonomousCommand("test path"));
  }
}
