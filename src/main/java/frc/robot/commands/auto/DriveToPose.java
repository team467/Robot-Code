package frc.robot.commands.auto;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.utils.GeomUtils;
import frc.lib.utils.TunableNumber;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * Command to drive the robot to a specified pose in a straight path. This is useful for very simple
 * autos or for alignment.
 */
public class DriveToPose extends Command {
  private final Drive drive;
  private final Supplier<Pose2d> poseSupplier;

  private boolean running = false;
  private final ProfiledPIDController driveController =
      new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));
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
        driveKp.initDefault(2.5);
        driveKd.initDefault(0.0);
        thetaKp.initDefault(3.0);
        thetaKd.initDefault(0.01);
        driveMaxVelocity.initDefault(Units.inchesToMeters(85));
        driveMaxAcceleration.initDefault(Units.inchesToMeters(450.0));
        thetaMaxVelocity.initDefault(Units.degreesToRadians(360.0));
        thetaMaxAcceleration.initDefault(Units.degreesToRadians(720.0));
        driveTolerance.initDefault(0.01);
        thetaTolerance.initDefault(Units.degreesToRadians(1.0));
      case ROBOT_SIMBOT:
        driveKp.initDefault(2.5);
        driveKd.initDefault(0.0);
        thetaKp.initDefault(3.0);
        thetaKd.initDefault(0.01);
        driveMaxVelocity.initDefault(Units.inchesToMeters(85));
        driveMaxAcceleration.initDefault(Units.inchesToMeters(450.0));
        thetaMaxVelocity.initDefault(Units.degreesToRadians(360.0));
        thetaMaxAcceleration.initDefault(Units.degreesToRadians(720.0));
        driveTolerance.initDefault(0.01);
        thetaTolerance.initDefault(Units.degreesToRadians(1.0));
      default:
        break;
    }
  }

  /**
   * Drives to the specified pose under full software control.
   *
   * @param drive The drive subsystem
   * @param pose The target pose
   */
  public DriveToPose(Drive drive, Pose2d pose) {
    this(drive, () -> pose);
  }

  public DriveToPose(Drive drive, double deltaXMeters, double deltaYMeters, double deltaThetaRad) {
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
                            : -deltaThetaRad))));
  }

  /**
   * Constructs a new DriveToPose command that drives the robot to a specified pose.
   *
   * @param drive The drive subsystem
   * @param poseSupplier A supplier that provides the target pose
   */
  public DriveToPose(Drive drive, Supplier<Pose2d> poseSupplier) {
    this.drive = drive;
    this.poseSupplier = poseSupplier;
    addRequirements(drive);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
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

    var currentPose = drive.getPose();
    driveController.reset(
        currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation()));
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
    var currentPose = drive.getPose();
    var targetPose = poseSupplier.get();

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
    var driveVelocity =
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
