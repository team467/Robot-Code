package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.utils.GeomUtils;
import frc.lib.utils.TunableNumber;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.Arrays;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class StolenJoystick extends Command {
  private final Drive drive;
  private final Supplier<Double> leftXSupplier;
  private final Supplier<Double> leftYSupplier;
  private final Supplier<Rotation2d> desiredRotation;
  private final Supplier<Boolean> robotRelativeOverride;

  private static final double TIME_TO_FULL_SPEED = 0.15;
  private final SlewRateLimiter leftXFilter = new SlewRateLimiter(1 / TIME_TO_FULL_SPEED);
  private final SlewRateLimiter leftYFilter = new SlewRateLimiter(1 / TIME_TO_FULL_SPEED);

  private TunableNumber rotationKP = new TunableNumber("StolenJoystick/KP", 1);
  private TunableNumber rotationKD = new TunableNumber("StolenJoystick/KD", 0);
  private final PIDController rotationPid;
  private final double MAX_ANGULAR_SPEED;

  public StolenJoystick(
      Drive drive,
      Supplier<Double> leftXSupplier,
      Supplier<Double> leftYSupplier,
      Supplier<Rotation2d> desiredRotation,
      Supplier<Boolean> robotRelativeOverride) {
    this.drive = drive;
    this.leftXSupplier = leftXSupplier;
    this.leftYSupplier = leftYSupplier;
    this.desiredRotation = desiredRotation;
    this.robotRelativeOverride = robotRelativeOverride;

    MAX_ANGULAR_SPEED =
        DriveConstants.MAX_LINEAR_SPEED
            / Arrays.stream(drive.getModuleTranslations())
                .map(Translation2d::getNorm)
                .max(Double::compare)
                .get();

    rotationPid = new PIDController(rotationKP.get(), 0.0, rotationKD.get());
    rotationPid.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive);
  }

  @Override
  public void execute() {
    // Update controllers if tunable numbers have changed
    if (Constants.tuningMode) {
      if (rotationKP.hasChanged(hashCode()) || rotationKD.hasChanged(hashCode())) {
        rotationPid.setPID(rotationKP.get(), 0, rotationKD.get());
      }
    }

    // Get values from double suppliers
    double leftX = leftXFilter.calculate(leftXSupplier.get());
    double leftY = leftYFilter.calculate(leftYSupplier.get());

    // Get direction and magnitude of linear axes
    double linearMagnitude = Math.hypot(leftX, leftY);
    Rotation2d linearDirection = new Rotation2d(leftX, leftY);

    // Apply deadband
    linearMagnitude = MathUtil.applyDeadband(linearMagnitude, 0.1);

    // Apply squaring
    linearMagnitude = Math.copySign(linearMagnitude * linearMagnitude, linearMagnitude);

    // Apply speed limits //TODO: Custom limits
    linearMagnitude *= 1.0;

    // Calcaulate new linear components
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(GeomUtils.transformFromTranslation(linearMagnitude, 0))
            .getTranslation();

    double angularVelocity =
        rotationPid.calculate(drive.getRotation().getRadians(), desiredRotation.get().getRadians());
    // Convert to meters per second
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * DriveConstants.MAX_LINEAR_SPEED,
            linearVelocity.getY() * DriveConstants.MAX_LINEAR_SPEED,
            angularVelocity);
    Logger.recordOutput("StolenJoystick/AngularVelocity", angularVelocity);

    // Convert from field relative
    if (robotRelativeOverride.get()) {
      Rotation2d driveRotation = drive.getPose().getRotation();
      if (DriverStation.getAlliance().isEmpty()
          || DriverStation.getAlliance().get() == Alliance.Red) {
        driveRotation = driveRotation.plus(new Rotation2d(Math.PI));
      }
      speeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              speeds.vxMetersPerSecond,
              speeds.vyMetersPerSecond,
              speeds.omegaRadiansPerSecond,
              driveRotation);
    }

    drive.runVelocity(speeds);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
