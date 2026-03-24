package frc.robot;

import static frc.robot.subsystems.shooter.ShooterConstants.CLOSE_HUB_SHOOTER_RPM;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
import frc.robot.subsystems.intake.rollers.IntakeRollers;
import frc.robot.subsystems.magicCarpet.MagicCarpet;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.ShooterLeadCompensator;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Orchestrator {
  private final double FRONT_HUB_OFFSET = Units.inchesToMeters(40.0);
  private final Drive drive;
  private final Shooter shooter;
  private final MagicCarpet magicCarpet;
  private final Indexer indexer;
  private final Intake intake;
  private final IntakeRollers rollers;
  private final RobotState robotState = RobotState.getInstance();
  private final ShooterLeadCompensator shooterLeadCompensator;
  private final CommandXboxController driverController;

  // Wraparound-safe angle filter
  private double filteredAngleRad = Double.NaN;
  private static final double ANGLE_ALPHA =
      0.15; // tune: lower = smoother, higher = more responsive

  // Filtering for lead compensator outputs
  private final LinearFilter targetXFilter = LinearFilter.singlePoleIIR(0.06, 0.02);
  private final LinearFilter targetYFilter = LinearFilter.singlePoleIIR(0.06, 0.02);
  private final LinearFilter distanceFilter = LinearFilter.singlePoleIIR(0.06, 0.02);

  public Orchestrator(
      Drive drive,
      MagicCarpet hopperBelt,
      Shooter shooter,
      Indexer indexer,
      Intake intake,
      IntakeRollers rollers,
      CommandXboxController driverController) {
    this.drive = drive;
    this.magicCarpet = hopperBelt;
    this.shooter = shooter;
    this.indexer = indexer;
    this.intake = intake;
    this.rollers = rollers;
    this.shooterLeadCompensator = new ShooterLeadCompensator(drive, shooter);
    this.driverController = driverController;
  }

  public Pose2d getShootWhileDrivingResultPose() {
    var shootWhileDrivingResult =
        shooterLeadCompensator.shootWhileDriving(
            AllianceFlipUtil.apply(Hub.innerCenterPoint.toTranslation2d()));
    return new Pose2d(
        targetXFilter.calculate(shootWhileDrivingResult.target().getX()),
        targetYFilter.calculate(shootWhileDrivingResult.target().getY()),
        Rotation2d.fromDegrees(0));
  }

  public DoubleSupplier getShootWhileDrivingResultDistance() {
    return () -> {
      var shootWhileDrivingResult =
          shooterLeadCompensator.shootWhileDriving(
              AllianceFlipUtil.apply(Hub.innerCenterPoint.toTranslation2d()));
      return shootWhileDrivingResult.distance();
    };
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
    var shootWhileDrivingResult =
        shooterLeadCompensator.shootWhileDriving(
            AllianceFlipUtil.apply(Hub.innerCenterPoint.toTranslation2d()));
    Logger.recordOutput(
        "Orchestrator/Target",
        new Pose2d(
            shootWhileDrivingResult.target().getX(),
            shootWhileDrivingResult.target().getY(),
            Rotation2d.fromDegrees(0)));
    Logger.recordOutput("Orchestrator/DistanceToHub", shootWhileDrivingResult.distance());
    Logger.recordOutput("Orchestrator/ShooterPosition", shooterLeadCompensator.shooterPose());
  }

  public Command driveToHub() {
    return new DriveToPose(
        drive,
        () -> {
          Pose2d hubApproachPose =
              AllianceFlipUtil.apply(
                  Hub.nearFace.transformBy(
                      new Transform2d(FRONT_HUB_OFFSET, 0.0, Rotation2d.fromDegrees(0.0))));
          Rotation2d sameAsAlignAndShootHeading =
              AllianceFlipUtil.apply(Hub.blueCenter)
                  .minus(hubApproachPose.getTranslation())
                  .getAngle();
          return new Pose2d(hubApproachPose.getTranslation(), sameAsAlignAndShootHeading);
        });
  }

  public Command alignToHub() {
    return new RotateToOrientation(
        drive,
        () -> {
          Pose2d hubApproachPose = AllianceFlipUtil.apply(Hub.nearFace);
          Rotation2d angle =
              hubApproachPose.getTranslation().minus(drive.getPose().getTranslation()).getAngle();
          return angle;
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
                shooter.calculateSetpoint(targetDistance).getAsDouble()));
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
        Commands.run(
            () -> {
              spinUpShooterDistance(this.getShootWhileDrivingResultDistance());
            }),
        DriveCommands.joystickDriveAtAngle(
            drive,
            driverController::getLeftX,
            driverController::getLeftY,
            () ->
                (getShootWhileDrivingResultPose()
                    .getTranslation()
                    .minus(drive.getPose().getTranslation())
                    .getAngle())));
  }

  public Command spinUpShooterWhileDriving() {
    return shooter.setTargetVelocityRadians(
        () -> shooter.calculateSetpoint(this.getShootWhileDrivingResultDistance()).getAsDouble());
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
    return Commands.parallel(rollers.intake(), preloadBalls());
  }
}
