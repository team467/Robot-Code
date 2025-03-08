package frc.robot.commands.auto;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.utils.GeomUtils;
import frc.lib.utils.TunableNumber;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class StraightDriveToPose extends Command {
  private final Drive drive;
  private final Supplier<Pose2d> poseSupplier;
  private boolean running = false;

  private final ProfiledPIDController driveController =
      new ProfiledPIDController(
          3.0,
          0,
          0.0,
          new Constraints(Units.inchesToMeters(85), Units.inchesToMeters(450.0 / 2))); // 90
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          5.0,
          0,
          0.0,
          new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(720.0 / 2)));

  private double driveErrorAbs;
  private double thetaErrorAbs;

  private static final TunableNumber driveKp = new TunableNumber("DriveToPose/DriveKp");
  private static final TunableNumber driveKd = new TunableNumber("DriveToPose/DriveKd");
  private static final TunableNumber thetaKp = new TunableNumber("DriveToPose/ThetaKp");
  private static final TunableNumber thetaKd = new TunableNumber("DriveToPose/ThetaKd");
  private static final TunableNumber driveMaxVelocity =
      new TunableNumber("DriveToPose/DriveMaxVelocity");
  private static final TunableNumber driveMaxAcceleration =
      new TunableNumber("DriveToPose/DriveMaxAcceleration");
  private static final TunableNumber thetaMaxVelocity =
      new TunableNumber("DriveToPose/ThetaMaxVelocity");
  private static final TunableNumber thetaMaxAcceleration =
      new TunableNumber("DriveToPose/ThetaMaxAcceleration");
  private static final TunableNumber driveTolerance =
      new TunableNumber("DriveToPose/DriveTolerance");
  private static final TunableNumber thetaTolerance =
      new TunableNumber("DriveToPose/ThetaTolerance");

  static {
    switch (Constants.getRobot()) {
      case ROBOT_2025_TEST:
      case ROBOT_2025_COMP:
        driveKp.initDefault(3.0);
        driveKd.initDefault(0.0);
        thetaKp.initDefault(3.0);
        thetaKd.initDefault(0.01);
        driveMaxVelocity.initDefault(Units.inchesToMeters(85));
        driveMaxAcceleration.initDefault(Units.inchesToMeters(450.0));
        thetaMaxVelocity.initDefault(Units.degreesToRadians(360.0));
        thetaMaxAcceleration.initDefault(Units.degreesToRadians(720.0));
        driveTolerance.initDefault(0.005);
        thetaTolerance.initDefault(Units.degreesToRadians(1.0));
      default:
        break;
    }
  }

  public StraightDriveToPose(
      double deltaXMeters, double deltaYMeters, double deltaThetaRad, Drive drive) {
    this(
        drive,
        () ->
            new Pose2d(
                new Translation2d(
                    drive.getPose().getTranslation().getX()
                        + (DriverStation.getAlliance().isEmpty()
                                || DriverStation.getAlliance().get() == Alliance.Blue
                            ? deltaXMeters
                            : -deltaXMeters),
                    drive.getPose().getTranslation().getY()
                        + (DriverStation.getAlliance().isEmpty()
                                || DriverStation.getAlliance().get() == Alliance.Blue
                            ? deltaYMeters
                            : -deltaYMeters)),
                new Rotation2d(
                    drive.getPose().getRotation().getRadians()
                        + (DriverStation.getAlliance().isEmpty()
                                || DriverStation.getAlliance().get() == Alliance.Blue
                            ? deltaThetaRad
                            : -deltaThetaRad))),
        driveTolerance.get());
  }

  public StraightDriveToPose(
      double deltaXMeters,
      double deltaYMeters,
      double deltaThetaRad,
      Drive drive,
      double driveTolerance) {
    this(
        drive,
        () ->
            new Pose2d(
                new Translation2d(
                    drive.getPose().getTranslation().getX()
                        + (DriverStation.getAlliance().isEmpty()
                                || DriverStation.getAlliance().get() == Alliance.Blue
                            ? deltaXMeters
                            : -deltaXMeters),
                    drive.getPose().getTranslation().getY()
                        + (DriverStation.getAlliance().isEmpty()
                                || DriverStation.getAlliance().get() == Alliance.Blue
                            ? deltaYMeters
                            : -deltaYMeters)),
                new Rotation2d(
                    drive.getPose().getRotation().getRadians()
                        + (DriverStation.getAlliance().isEmpty()
                                || DriverStation.getAlliance().get() == Alliance.Blue
                            ? deltaThetaRad
                            : -deltaThetaRad))),
        driveTolerance);
  }

  public StraightDriveToPose(Pose2d targetPose, Drive drive) {
    this(drive, () -> targetPose, driveTolerance.get());
  }

  public StraightDriveToPose(Drive drive, Pose2d targetPose) {
    this(drive, () -> targetPose, driveTolerance.get());
  }

  public StraightDriveToPose(Drive drive, Supplier<Pose2d> targetPoseSupplier) {
    this(drive, targetPoseSupplier, driveTolerance.get());
  }

  public StraightDriveToPose(
      Drive drive, Supplier<Pose2d> targetPoseSupplier, double driveTolerance) {
    this.drive = drive;
    this.poseSupplier = targetPoseSupplier;
    addRequirements(drive);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    driveController.setTolerance(driveTolerance);
  }

  @Override
  public void initialize() {
    RobotState.visionEnabled = false;
    Pose2d currentPose = drive.getPose();
    Pose2d targetPose = poseSupplier.get();

    // Reset all controllers
    driveController.setP(driveKp.get());
    driveController.setD(driveKd.get());
    driveController.setConstraints(
        new TrapezoidProfile.Constraints(driveMaxVelocity.get(), driveMaxAcceleration.get()));
    driveController.setTolerance(driveTolerance.get());
    thetaController.setP(thetaKp.get());
    thetaController.setD(thetaKd.get());
    thetaController.setConstraints(
        new TrapezoidProfile.Constraints(thetaMaxVelocity.get(), thetaMaxAcceleration.get()));
    thetaController.setTolerance(thetaTolerance.get());

    driveController.reset(currentPose.getTranslation().getDistance(targetPose.getTranslation()));
    thetaController.reset(currentPose.getRotation().getRadians());
  }

  @Override
  public void execute() {
    running = true;

    // Update from tunable numbers
    if (driveKp.hasChanged(hashCode())
        || driveKd.hasChanged(hashCode())
        || thetaKp.hasChanged(hashCode())
        || thetaKd.hasChanged(hashCode())
        || driveMaxVelocity.hasChanged(hashCode())
        || driveMaxAcceleration.hasChanged(hashCode())
        || thetaMaxVelocity.hasChanged(hashCode())
        || thetaMaxAcceleration.hasChanged(hashCode())
        || driveTolerance.hasChanged(hashCode())
        || thetaTolerance.hasChanged(hashCode())) {
      driveController.setP(driveKp.get());
      driveController.setD(driveKd.get());
      driveController.setConstraints(
          new TrapezoidProfile.Constraints(driveMaxVelocity.get(), driveMaxAcceleration.get()));
      driveController.setTolerance(driveTolerance.get());
      thetaController.setP(thetaKp.get());
      thetaController.setD(thetaKd.get());
      thetaController.setConstraints(
          new TrapezoidProfile.Constraints(thetaMaxVelocity.get(), thetaMaxAcceleration.get()));
      thetaController.setTolerance(thetaTolerance.get());
    }

    // Get current and target pose
    Pose2d currentPose = drive.getPose();
    Pose2d targetPose = poseSupplier.get();
    Logger.recordOutput("StraightDriveToPose/CurrentPose", currentPose);
    Logger.recordOutput("StraightDriveToPose/TargetPose", targetPose);

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
    RobotState.visionEnabled = true;
    running = false;
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return atGoal();
  }

  /** Checks if the robot is stopped at the final pose. */
  public boolean atGoal() {
    return running && driveController.atGoal() && thetaController.atGoal();
  }

  /** Checks if the robot pose is within the allowed drive and theta tolerances. */
  public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
    return running
        && Math.abs(driveErrorAbs) < driveTolerance
        && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
  }

  /** Returns whether the command is actively running. */
  @AutoLogOutput
  public boolean isRunning() {
    return running;
  }
}
