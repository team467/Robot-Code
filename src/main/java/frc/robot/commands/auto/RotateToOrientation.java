package frc.robot.commands.auto;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class RotateToOrientation extends Command {
  private final Drive drive;
  private final Supplier<Rotation2d> poseSupplier;
  //  private final ProfiledPIDController driveController =
  //      new ProfiledPIDController(
  //          2.5, 0, 0.0, new Constraints(Units.inchesToMeters(150), Units.inchesToMeters(450.0)));
  //  private final ProfiledPIDController thetaController =
  //      new ProfiledPIDController(
  //          7.0, 0, 0.0, new Constraints(Units.degreesToRadians(360),
  // Units.degreesToRadians(720.0)));

  private final ProfiledPIDController thetaController =
      new ProfiledPIDController( // TODO: Tune this
          2.5, 0, .01, new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(720.0)));

  private Rotation2d targetPose;
  private double thetaErrorAbs;

  private static final double DRIVE_TOLERANCE = 0.005;
  private static final double THETA_TOLERANCE = Units.degreesToRadians(1.0);

  public RotateToOrientation(Rotation2d targetPose, Drive drive) {
    this(drive, () -> targetPose, DRIVE_TOLERANCE);
  }

  public RotateToOrientation(Drive drive, Rotation2d targetPose) {
    this(drive, () -> targetPose, DRIVE_TOLERANCE);
  }

  public RotateToOrientation(Drive drive, Supplier<Rotation2d> targetPoseSupplier) {
    this(drive, targetPoseSupplier, DRIVE_TOLERANCE);
  }

  public RotateToOrientation(
      Drive drive, Supplier<Rotation2d> targetPoseSupplier, double driveTolerance) {
    this.drive = drive;
    this.poseSupplier = targetPoseSupplier;
    addRequirements(drive);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    Pose2d currentPose = drive.getPose();
    targetPose = poseSupplier.get();
    Logger.recordOutput("StraightDriveToPose/TargetPose", targetPose);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(THETA_TOLERANCE);
    thetaController.reset(currentPose.getRotation().getRadians());
  }

  @Override
  public void execute() {
    Pose2d currentPose = drive.getPose();

    // Calculate theta speed
    double thetaVelocity =
        thetaController.calculate(currentPose.getRotation().getRadians(), targetPose.getRadians());
    thetaErrorAbs = Math.abs(currentPose.getRotation().minus(targetPose).getRadians());
    if (thetaController.atGoal()) thetaVelocity = 0.0;

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, thetaVelocity, currentPose.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return thetaController.atGoal();
  }
}
