package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.utils.GeomUtils;
import frc.robot.RobotConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

public class DriveWithJoysticks extends CommandBase {
  private final Drive drive;
  private final Supplier<Double> leftXSupplier;
  private final Supplier<Double> leftYSupplier;
  private final Supplier<Double> rightXSupplier;
  private final Supplier<Boolean> robotRelativeOverride;

  private static final double DEADBAND = 0.1;

  public DriveWithJoysticks(
      Drive drive,
      Supplier<Double> leftXSupplier,
      Supplier<Double> leftYSupplier,
      Supplier<Double> rightXSupplier,
      Supplier<Boolean> robotRelativeOverride) {
    this.drive = drive;
    this.leftXSupplier = leftXSupplier;
    this.leftYSupplier = leftYSupplier;
    this.rightXSupplier = rightXSupplier;
    this.robotRelativeOverride = robotRelativeOverride;

    addRequirements(drive);
  }

  @Override
  public void execute() {
    // Get values from double suppliers
    double leftX = leftXSupplier.get();
    double leftY = leftYSupplier.get();
    double rightX = rightXSupplier.get();

    // Get direction and magnitude of linear axes
    double linearMagnitude = Math.hypot(leftX, leftY);
    Rotation2d linearDirection = new Rotation2d(leftX, leftY);

    // Apply deadband
    linearMagnitude = MathUtil.applyDeadband(linearMagnitude, 0.1);
    rightX = MathUtil.applyDeadband(rightX, 0.1);

    // Apply squaring
    linearMagnitude = Math.copySign(linearMagnitude * linearMagnitude, linearMagnitude);
    rightX = Math.copySign(rightX * rightX, rightX);

    // Apply speed limits //TODO: Custom limits
    linearMagnitude *= 1.0;
    rightX *= 1.0;

    // Calcaulate new linear components
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(GeomUtils.transformFromTranslation(linearMagnitude, 0))
            .getTranslation();

    // Convert to meters per second
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * RobotConstants.get().maxLinearSpeed(),
            linearVelocity.getY() * RobotConstants.get().maxLinearSpeed(),
            rightX * RobotConstants.get().maxAngularSpeed());

    // Convert from field relative
    if (robotRelativeOverride.get()) {
      speeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              speeds.vxMetersPerSecond,
              speeds.vyMetersPerSecond,
              speeds.omegaRadiansPerSecond,
              drive.getPose().getRotation());
    }

    drive.runVelocity(speeds);
  }
}
