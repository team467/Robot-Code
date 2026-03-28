package frc.robot;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.utils.AllianceFlipUtil;
import frc.robot.FieldConstants.Hub;
import frc.robot.commands.auto.DriveToPose;
import frc.robot.commands.auto.RotateToOrientation;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.magicCarpet.MagicCarpet;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Orchestrator {
  private static final double FRONT_HUB_OFFSET = Units.inchesToMeters(40.0);
  private static final double PHASE_DELAY_SECONDS = 0.03;
  private static final int CONVERGENCE_ITERATIONS = 20;
  private static final double ANGLE_ALPHA = 0.15;

  private final Drive drive;
  private final Shooter shooter;
  private final MagicCarpet magicCarpet;
  private final Indexer indexer;
  private final Intake intake;
  private final CommandXboxController driverController;

  private double filteredAngleRad = Double.NaN;
  private final LinearFilter targetXFilter = LinearFilter.singlePoleIIR(0.06, 0.02);
  private final LinearFilter targetYFilter = LinearFilter.singlePoleIIR(0.06, 0.02);

  public Orchestrator(
      Drive drive,
      MagicCarpet hopperBelt,
      Shooter shooter,
      Indexer indexer,
      Intake intake,
      CommandXboxController driverController) {
    this.drive = drive;
    this.magicCarpet = hopperBelt;
    this.shooter = shooter;
    this.indexer = indexer;
    this.intake = intake;
    this.driverController = driverController;
  }

  private Translation2d getShooterPosition(Pose2d robotPose) {
    Translation2d rotatedOffset =
        kShooterOffsetFromRobotCenter.getTranslation().rotateBy(robotPose.getRotation());
    return robotPose.getTranslation().plus(rotatedOffset);
  }

  private record LeadResult(Pose2d target, double distance, double timeOfFlight) {}

  private LeadResult computeLeadCompensation(Translation2d targetPosition) {
    Pose2d robotPose = drive.getPose();
    ChassisSpeeds speeds = drive.getChassisSpeeds();

    Pose2d compensatedPose =
        robotPose.exp(
            new Twist2d(
                speeds.vxMetersPerSecond * PHASE_DELAY_SECONDS,
                speeds.vyMetersPerSecond * PHASE_DELAY_SECONDS,
                speeds.omegaRadiansPerSecond * PHASE_DELAY_SECONDS));

    Translation2d shooterPos = getShooterPosition(compensatedPose);
    Translation2d fieldVelocity =
        new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

    Translation2d aimPoint = targetPosition;
    double timeOfFlight = 0.0;

    for (int i = 0; i < CONVERGENCE_ITERATIONS; i++) {
      double distance = aimPoint.minus(shooterPos).getNorm();
      timeOfFlight = timeOfFlightMap.get(distance);
      aimPoint = targetPosition.minus(fieldVelocity.times(timeOfFlight));
    }

    double finalDistance = aimPoint.minus(shooterPos).getNorm();

    Logger.recordOutput("LeadComp/AimPoint", new Pose2d(aimPoint, Rotation2d.kZero));
    Logger.recordOutput("LeadComp/Distance", finalDistance);
    Logger.recordOutput("LeadComp/TOF", timeOfFlight);

    return new LeadResult(
        new Pose2d(aimPoint, aimPoint.minus(shooterPos).getAngle()), finalDistance, timeOfFlight);
  }

  public Pose2d getShootWhileDrivingResultPose() {
    var result =
        computeLeadCompensation(AllianceFlipUtil.apply(Hub.innerCenterPoint.toTranslation2d()));
    return new Pose2d(
        targetXFilter.calculate(result.target().getX()),
        targetYFilter.calculate(result.target().getY()),
        Rotation2d.kZero);
  }

  public DoubleSupplier getShootWhileDrivingResultDistance() {
    return () ->
        computeLeadCompensation(AllianceFlipUtil.apply(Hub.innerCenterPoint.toTranslation2d()))
            .distance();
  }

  public DoubleSupplier getHubDistance() {
    return () ->
        AllianceFlipUtil.apply(Hub.innerCenterPoint.toTranslation2d())
            .getDistance(drive.getPose().getTranslation());
  }

  private Rotation2d filteredHubAngle(Rotation2d raw) {
    if (Double.isNaN(filteredAngleRad)) {
      filteredAngleRad = raw.getRadians();
      return raw;
    }
    double error = raw.minus(Rotation2d.fromRadians(filteredAngleRad)).getRadians();
    filteredAngleRad += ANGLE_ALPHA * error;
    return Rotation2d.fromRadians(filteredAngleRad);
  }

  public void orchestratorPeriodic() {
    var result =
        computeLeadCompensation(AllianceFlipUtil.apply(Hub.innerCenterPoint.toTranslation2d()));
    Logger.recordOutput("Orchestrator/Target", result.target());
    Logger.recordOutput("Orchestrator/DistanceToHub", result.distance());
    Logger.recordOutput(
        "Orchestrator/ShooterPosition",
        new Pose2d(getShooterPosition(drive.getPose()), Rotation2d.kZero));
  }

  public Command driveToHub() {
    return new DriveToPose(
        drive,
        () -> {
          Pose2d hubApproachPose =
              AllianceFlipUtil.apply(
                  Hub.nearFace.transformBy(
                      new Transform2d(FRONT_HUB_OFFSET, 0.0, Rotation2d.fromDegrees(0.0))));
          Rotation2d heading =
              AllianceFlipUtil.apply(Hub.blueCenter)
                  .minus(hubApproachPose.getTranslation())
                  .getAngle();
          return new Pose2d(hubApproachPose.getTranslation(), heading);
        });
  }

  public Command alignToHub() {
    return new RotateToOrientation(
        drive,
        () -> {
          Pose2d hubApproachPose = AllianceFlipUtil.apply(Hub.nearFace);
          return hubApproachPose
              .getTranslation()
              .minus(drive.getPose().getTranslation())
              .getAngle();
        });
  }

  public Command feedUp() {
    return Commands.repeatingSequence(
            preloadBalls().until(() -> RobotState.getInstance().shooterAtSpeed),
            Commands.parallel(magicCarpet.run(), indexer.run())
                .until(() -> !RobotState.getInstance().shooterAtSpeed))
        .onlyIf(() -> shooter.getSetpoint() > 0)
        .onlyWhile(() -> shooter.getSetpoint() > 0)
        .withName("feedUp");
  }

  public Command spinUpShooterTest() {
    SmartDashboard.putNumber("Shooter/TestRPM", CLOSE_HUB_SHOOTER_RPM);
    return shooter
        .setTargetVelocityRadians(
            () ->
                Units.rotationsPerMinuteToRadiansPerSecond(
                    SmartDashboard.getNumber("Shooter/TestRPM", 1000.0)))
        .withName("spinUpShooterTest");
  }

  public Command spinUpShooterDistance(DoubleSupplier targetDistance) {
    return shooter.setTargetVelocityRadians(
        () ->
            Units.rotationsPerMinuteToRadiansPerSecond(
                shooterRpmMap.get(targetDistance.getAsDouble())));
  }

  public Command spinUpShooterHub() {
    return shooter.setTargetVelocityRadians(
        Units.rotationsPerMinuteToRadiansPerSecond(CLOSE_HUB_SHOOTER_RPM));
  }

  public Command spinUpShooter(double velocityRPM) {
    return shooter.setTargetVelocityRadians(
        Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM));
  }

  public Command preloadBalls() {
    return Commands.parallel(magicCarpet.run(), indexer.runPreloadSpeeds())
        .onlyWhile(() -> !RobotState.getInstance().indexerHasFuel)
        .onlyIf(() -> !RobotState.getInstance().indexerHasFuel)
        .until(() -> RobotState.getInstance().indexerHasFuel)
        .andThen(indexer.stop().withTimeout(0.01));
  }

  public Command driveShootAtAngle() {
    return Commands.parallel(
        Commands.run(() -> spinUpShooterDistance(this.getShootWhileDrivingResultDistance())),
        DriveCommands.joystickDriveAtAngle(
            drive,
            driverController::getLeftX,
            driverController::getLeftY,
            () ->
                getShootWhileDrivingResultPose()
                    .getTranslation()
                    .minus(drive.getPose().getTranslation())
                    .getAngle()));
  }

  public Command spinUpShooterWhileDriving() {
    return shooter.setTargetVelocityRadians(
        () ->
            Units.rotationsPerMinuteToRadiansPerSecond(
                shooterRpmMap.get(getShootWhileDrivingResultDistance().getAsDouble())));
  }

  public Command aimToHub() {
    return new DriveToPose(
        drive,
        () ->
            new Pose2d(
                drive.getPose().getX(),
                drive.getPose().getY(),
                AllianceFlipUtil.apply(Hub.blueCenter)
                    .minus(drive.getPose().getTranslation())
                    .getAngle()));
  }

  public Command preloadWhileIntaking() {
    return Commands.parallel(intake.intake(), preloadBalls());
  }
}
