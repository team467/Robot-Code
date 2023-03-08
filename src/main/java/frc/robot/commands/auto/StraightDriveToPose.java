package frc.robot.commands.auto;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.utils.GeomUtils;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class StraightDriveToPose extends CommandBase {
  private final Drive drive;
  private final Supplier<Pose2d> poseSupplier;
  //  private final ProfiledPIDController driveController =
  //      new ProfiledPIDController(
  //          2.5, 0, 0.0, new Constraints(Units.inchesToMeters(150), Units.inchesToMeters(450.0)));
  //  private final ProfiledPIDController thetaController =
  //      new ProfiledPIDController(
  //          7.0, 0, 0.0, new Constraints(Units.degreesToRadians(360),
  // Units.degreesToRadians(720.0)));

  private final ProfiledPIDController driveController =
      new ProfiledPIDController(
          1.2, 0, 0.0, new Constraints(Units.inchesToMeters(150), Units.inchesToMeters(450.0)));
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          4.0, 0, 0.5, new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(720.0)));

  private static final double DRIVE_TOLERANCE = 0.01;
  private static final double THETA_TOLERANCE = Units.degreesToRadians(2.0);

  public StraightDriveToPose(
      double deltaXMeters, double deltaYMeters, double deltaThetaRad, Drive drive) {
    this(
        drive,
        () ->
            new Pose2d(
                new Translation2d(
                    drive.getPose().getTranslation().getX() + deltaXMeters,
                    drive.getPose().getTranslation().getY() + deltaYMeters),
                drive.getPose().getRotation().plus(new Rotation2d(deltaThetaRad))));
  }

  public StraightDriveToPose(Drive drive, Pose2d targetPose) {
    this(drive, () -> targetPose);
  }

  public StraightDriveToPose(Drive drive, Supplier<Pose2d> targetPoseSupplier) {
    this.drive = drive;
    this.poseSupplier = targetPoseSupplier;
    addRequirements(drive);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    Pose2d currentPose = drive.getPose();
    driveController.setTolerance(DRIVE_TOLERANCE);
    thetaController.setTolerance(THETA_TOLERANCE);
    driveController.reset(
        currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation()));
    thetaController.reset(
        currentPose.getRotation().minus(poseSupplier.get().getRotation()).getRadians());
  }

  @Override
  public void execute() {
    Pose2d currentPose = drive.getPose();
    Pose2d targetPose = poseSupplier.get();

    // Command speeds
    double driveVelocityScalar =
        driveController.calculate(
            currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation()), 0.0);
    double thetaVelocity =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    Logger.getInstance().recordOutput("ThetaSetpoint", targetPose.getRotation().getRadians());
    Logger.getInstance().recordOutput("ThetaCurrent", currentPose.getRotation().getRadians());
    Logger.getInstance().recordOutput("ThetaFix", thetaVelocity);
    if (driveController.atGoal()) driveVelocityScalar = 0.0;
    if (thetaController.atGoal()) thetaVelocity = 0.0;
    Pose2d driveVelocity =
        new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(GeomUtils.transformFromTranslation(driveVelocityScalar, 0.0));
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return driveController.atGoal() && thetaController.atGoal();
  }
}
