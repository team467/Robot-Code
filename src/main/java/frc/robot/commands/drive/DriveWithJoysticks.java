package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.utils.GeomUtils;
import frc.lib.utils.RobotOdometry;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.Arrays;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveWithJoysticks extends Command {
  private final Drive drive;
  private final Supplier<Double> leftXSupplier;
  private final Supplier<Double> leftYSupplier;
  private final Supplier<Double> rightXSupplier;
  private final Supplier<Boolean> robotRelativeOverride;

  private static final double DEADBAND = 0.1;
  private static final double TIME_TO_FULL_SPEED = 0.15;
  private final SlewRateLimiter leftXFilter = new SlewRateLimiter(1 / TIME_TO_FULL_SPEED);
  private final SlewRateLimiter leftYFilter = new SlewRateLimiter(1 / TIME_TO_FULL_SPEED);
  private final SlewRateLimiter rightXFilter = new SlewRateLimiter(1 / TIME_TO_FULL_SPEED);

  private final double MAX_ANGULAR_SPEED;

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

    MAX_ANGULAR_SPEED =
        DriveConstants.MAX_LINEAR_SPEED
            / Arrays.stream(drive.getModuleTranslations())
                .map(Translation2d::getNorm)
                .max(Double::compare)
                .get();

    addRequirements(drive);
  }

  @Override
  public void execute() {
    // Get values from double suppliers
    double leftX = leftXFilter.calculate(leftXSupplier.get());
    double leftY = leftYFilter.calculate(leftYSupplier.get());
    double rightX = rightXFilter.calculate(rightXSupplier.get());
    //    double leftX = leftXSupplier.get();
    //    double leftY = leftYSupplier.get();
    //    double rightX = rightXSupplier.get();

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
    ChassisSpeeds speeds;
    if (!robotRelativeOverride.get()) {
      speeds =
          new ChassisSpeeds(
              linearVelocity.getX() * DriveConstants.MAX_LINEAR_SPEED,
              linearVelocity.getY() * DriveConstants.MAX_LINEAR_SPEED,
              rightX * DriveConstants.MAX_LINEAR_SPEED);
    } else {
      if (DriverStation.getAlliance().isPresent()
          && DriverStation.getAlliance().get() == Alliance.Red) {
        Logger.recordOutput("Drive/FlipAlliance", true);
        linearVelocity = linearVelocity.rotateBy(Rotation2d.fromRadians(Math.PI));
      } else {
        Logger.recordOutput("Drive/FlipAlliance", false);
      }
      speeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              linearVelocity.getX() * DriveConstants.MAX_LINEAR_SPEED,
              linearVelocity.getY() * DriveConstants.MAX_LINEAR_SPEED,
              rightX * DriveConstants.MAX_LINEAR_SPEED,
              RobotOdometry.getInstance().getLatestPose().getRotation());
    }
    drive.runVelocity(speeds);

    //    // Convert to meters per second
    //    ChassisSpeeds speeds =
    //        new ChassisSpeeds(
    //            linearVelocity.getX() * DriveConstants.MAX_LINEAR_SPEED,
    //            linearVelocity.getY() * DriveConstants.MAX_LINEAR_SPEED,
    //            rightX * MAX_ANGULAR_SPEED);
    //
    //    // Convert from field relative
    //    if (robotRelativeOverride.get()) {
    //      Rotation2d driveRotation = drive.getPose().getRotation();
    //      if (DriverStation.getAlliance().isEmpty()
    //          || DriverStation.getAlliance().get() == Alliance.Red) {
    //        driveRotation = driveRotation.plus(new Rotation2d(Math.PI));
    //      }
    //      speeds =
    //          ChassisSpeeds.fromFieldRelativeSpeeds(
    //              speeds.vxMetersPerSecond,
    //              speeds.vyMetersPerSecond,
    //              speeds.omegaRadiansPerSecond,
    //              driveRotation);
    //    }
    //
    //    drive.runVelocity(speeds);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
