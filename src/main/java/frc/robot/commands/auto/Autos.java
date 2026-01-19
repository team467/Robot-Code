package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.utils.AllianceFlipUtil;
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

  public Command CenterLeft() {
    Supplier<Pose2d> intakeStart =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(6.710, 5.009, new Rotation2d(Units.degreesToRadians(-147.288))));
    Supplier<Pose2d> intakeEnd =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(7.963, 6.181, new Rotation2d(Units.degreesToRadians(-147.288))));
    Supplier<Pose2d> closestLeftShot =
        () ->
            AllianceFlipUtil.apply(
                new Pose2d(3.229, 5.331, new Rotation2d(Units.degreesToRadians(-55.222))));
    return Commands.sequence(
        Commands.runOnce(() -> drive.setPose(center.get())),
        drive.getAutonomousCommand("CL out").withTimeout(2.64),
        new StraightDriveToPose(intakeStart.get(), drive).withTimeout(0.5),
        new StraightDriveToPose(intakeEnd.get(), drive).withTimeout(1.3),
        new StraightDriveToPose(closestLeftShot.get(), drive).withTimeout(3.4),
        new StraightDriveToPose(climb.get(), drive).withTimeout(2.0));
  }
}
