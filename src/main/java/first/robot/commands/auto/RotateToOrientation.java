package first.robot.commands.auto;

import first.robot.mechanisms.swerve.Drive;
import java.util.Objects;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.TrapezoidProfile.Constraints;
import org.wpilib.math.util.Units;

public class RotateToOrientation implements Command {
  private final Drive drive;
  private final Supplier<Rotation2d> rotationSupplier;

  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          2.5, 0, .01, new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(720.0)));

  private static final double DRIVE_TOLERANCE = 0.005;
  private static final double THETA_TOLERANCE = Units.degreesToRadians(1.0);

  public RotateToOrientation(Rotation2d targetPose, Drive drive) {
    this(drive, () -> targetPose, DRIVE_TOLERANCE);
  }

  public RotateToOrientation(Drive drive, Rotation2d targetPose) {
    this(drive, () -> targetPose, DRIVE_TOLERANCE);
  }

  public RotateToOrientation(Drive drive, Pose2d targetPose) {
    this(drive, () -> headingTo(drive, targetPose), DRIVE_TOLERANCE);
  }

  public RotateToOrientation(Drive drive, Supplier<Pose2d> targetPoseSupplier) {
    this(
        drive,
        () ->
            headingTo(
                drive,
                Objects.requireNonNull(targetPoseSupplier.get(), "targetPoseSupplier.get()")),
        DRIVE_TOLERANCE);
  }

  public RotateToOrientation(
      Drive drive, Supplier<Rotation2d> targetPoseSupplier, double driveTolerance) {
    this.drive = drive;
    this.rotationSupplier = Objects.requireNonNull(targetPoseSupplier, "targetPoseSupplier");
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  private static Rotation2d headingTo(Drive drive, Pose2d target) {
    return target
        .getTranslation()
        .minus(drive.getPose().getTranslation())
        .getAngle()
        .orElse(drive.getRotation());
  }

  @Override
  public void run(Coroutine coroutine) {
    Pose2d currentPose = drive.getPose();
    Rotation2d targetPose = rotationSupplier.get();
    Logger.recordOutput("RotateToOrientation/TargetPose", targetPose);
    thetaController.setTolerance(THETA_TOLERANCE);
    thetaController.reset(currentPose.getRotation().getRadians());

    while (true) {
      currentPose = drive.getPose();
      targetPose = rotationSupplier.get();
      double thetaVelocity =
          thetaController.calculate(currentPose.getRotation().getRadians(), targetPose.getRadians());
      if (thetaController.atGoal()) {
        break;
      }
      drive.runVelocity(
          new ChassisVelocities(0, 0, thetaVelocity).toRobotRelative(currentPose.getRotation()));
      coroutine.yield();
    }
    drive.stop();
  }

  @Override
  public void onCancel() {
    drive.stop();
  }

  @Override
  public String name() {
    return "RotateToOrientation";
  }

  @Override
  public Set<Mechanism> requirements() {
    return Set.of(drive);
  }
}
