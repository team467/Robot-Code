package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.utils.AllianceFlipUtil;
import frc.lib.utils.GeomUtils;
import frc.lib.utils.RobotOdometry;
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
  private final Supplier<Pose2d> robotOrientation;
  private final Translation2d targetPosition;
  private final Supplier<Boolean> robotRelativeOverride;

  private static final double TIME_TO_FULL_SPEED = 0.15;
  private final SlewRateLimiter leftXFilter = new SlewRateLimiter(1 / TIME_TO_FULL_SPEED);
  private final SlewRateLimiter leftYFilter = new SlewRateLimiter(1 / TIME_TO_FULL_SPEED);

  private TunableNumber rotationKP = new TunableNumber("StolenJoystick/KP", 5); // 0.5
  private TunableNumber rotationKD = new TunableNumber("StolenJoystick/KD", 0);
  private final PIDController rotationPid;
  private final double MAX_ANGULAR_SPEED;

  public StolenJoystick(
      Drive drive,
      Supplier<Double> leftXSupplier,
      Supplier<Double> leftYSupplier,
      Supplier<Pose2d> robotOrientation,
      Translation2d targetPosition,
      Supplier<Boolean> robotRelativeOverride) {
    this.drive = drive;
    this.leftXSupplier = leftXSupplier;
    this.leftYSupplier = leftYSupplier;
    this.robotOrientation = robotOrientation;
    this.targetPosition = targetPosition;
    this.robotRelativeOverride = robotRelativeOverride;

    MAX_ANGULAR_SPEED =
        DriveConstants.MAX_LINEAR_SPEED
            / Arrays.stream(drive.getModuleTranslations())
                .map(Translation2d::getNorm)
                .max(Double::compare)
                .get();

    rotationPid = new PIDController(rotationKP.get(), 0.0, rotationKD.get());
    rotationPid.enableContinuousInput(-Math.PI, Math.PI);
    rotationPid.setTolerance(Units.degreesToRadians(5));

    addRequirements(drive);
    setName("StolenJoystick");
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

    // Calculate the target angle based on the current position of the robot and the speaker
    Rotation2d targetAngle =
        new Pose2d(
                robotOrientation.get().getTranslation(),
                AllianceFlipUtil.apply(targetPosition)
                    .minus(robotOrientation.get().getTranslation())
                    .getAngle()
                    .minus(Rotation2d.fromDegrees(180)))
            .getRotation();
    Logger.recordOutput("Drive/TargetAngle", targetAngle);
    double angularVelocity =
        rotationPid.calculate(
            robotOrientation.get().getRotation().getRadians(), targetAngle.getRadians());
    // Convert to meters per second
    ChassisSpeeds speeds;
    Logger.recordOutput("StolenJoystick/AngularVelocity", angularVelocity);

    if (AllianceFlipUtil.shouldFlip()) {
      Logger.recordOutput("Drive/FlipAlliance", true);
      linearVelocity = linearVelocity.rotateBy(Rotation2d.fromRadians(Math.PI));
    } else {
      Logger.recordOutput("Drive/FlipAlliance", false);
    }
    speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            linearVelocity.getX() * DriveConstants.MAX_LINEAR_SPEED,
            linearVelocity.getY() * DriveConstants.MAX_LINEAR_SPEED,
            angularVelocity,
            RobotOdometry.getInstance().getPoseEstimator().getEstimatedPosition().getRotation());

    drive.runVelocity(speeds);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
