package frc.robot.commands.auto;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.utils.GeomUtils;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

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
          //          2.0, 0, 0.0, new Constraints(Units.inchesToMeters(150),
          // Units.inchesToMeters(450.0)));
          2.0,
          0,
          0.0,
          new Constraints(Units.inchesToMeters(85), Units.inchesToMeters(450.0))); // 90
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          5.0, 0, 0.0, new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(720.0)));

  private Pose2d targetPose;
  private double driveErrorAbs;
  private double thetaErrorAbs;

  private static final double DRIVE_TOLERANCE = 0.01;
  private static final double THETA_TOLERANCE = Units.degreesToRadians(1.0);

  public StraightDriveToPose(
      double deltaXMeters, double deltaYMeters, double deltaThetaRad, Drive drive) {
    this(
        drive,
        () ->
            new Pose2d(
                new Translation2d(
                    drive.getPose().getTranslation().getX()
                        + (DriverStation.getAlliance().get() == Alliance.Blue
                            ? deltaXMeters
                            : -deltaXMeters),
                    drive.getPose().getTranslation().getY()
                        + (DriverStation.getAlliance().get() == Alliance.Blue
                            ? deltaYMeters
                            : -deltaYMeters)),
                new Rotation2d(
                    drive.getPose().getRotation().getRadians()
                        + (DriverStation.getAlliance().get() == Alliance.Blue
                            ? deltaThetaRad
                            : -deltaThetaRad))));
  }

  public StraightDriveToPose(Pose2d targetPose, Drive drive) {
    this(drive, () -> targetPose);
  }

  private StraightDriveToPose(Drive drive, Supplier<Pose2d> targetPoseSupplier) {
    this.drive = drive;
    this.poseSupplier = targetPoseSupplier;
    addRequirements(drive);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    Pose2d currentPose = drive.getPose();
    targetPose = poseSupplier.get();
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    driveController.setTolerance(DRIVE_TOLERANCE);
    thetaController.setTolerance(THETA_TOLERANCE);
    driveController.reset(currentPose.getTranslation().getDistance(targetPose.getTranslation()));
    thetaController.reset(currentPose.getRotation().getRadians());
  }

  @Override
  public void execute() {
    Pose2d currentPose = drive.getPose();

    // Calculate drive speed
    double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    driveErrorAbs = currentDistance;
    double driveVelocityScalar = driveController.calculate(driveErrorAbs, 0.0);
    if (driveController.atGoal()) driveVelocityScalar = 0.0;

    // Calculate theta speed
    double thetaVelocity =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    thetaErrorAbs =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
    if (thetaController.atGoal()) thetaVelocity = 0.0;

    // Command speeds
    Translation2d driveVelocity =
        new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(GeomUtils.transformFromTranslation(driveVelocityScalar, 0.0))
            .getTranslation();
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
