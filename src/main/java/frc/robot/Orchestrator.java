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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
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
import frc.robot.util.Zone;
import frc.robot.util.Zone.Tuple2d;
import java.util.Map;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Orchestrator {
  private final Pose2d BBumpClosePose =
      new Pose2d(4.415811061859131, 5.599213600158691, new Rotation2d());
  private final Pose2d BBumpFarPose =
      new Pose2d(11.334761619567871, 5.599213600158691, new Rotation2d());

  public enum ZoneId {
    NONE,
    ZONE_1,
    ZONE_2,
    ZONE_3,
    ZONE_4
  }

  private final double FRONT_HUB_OFFSET = Units.inchesToMeters(40.0);
  private final Drive drive;
  private final Shooter shooter;
  private final MagicCarpet magicCarpet;
  private final Indexer indexer;
  private final Intake intake;
  private final IntakeRollers rollers;
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

  private final Zone zone1;
  private final Zone zone2;
  private final Zone zone3;
  private final Zone zone4;

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

    this.zone1 = new Zone(drive::getPose);
    this.zone2 = new Zone(drive::getPose);
    this.zone3 = new Zone(drive::getPose);
    this.zone4 = new Zone(drive::getPose);
    zone1.initializeZone(new Tuple2d(0.0, 0.0), new Tuple2d(3.993527889251709, 8.100430488586426));
    zone2.initializeZone(
        new Tuple2d(4.307533264160156, 8.04629135131836), new Tuple2d(11.9086332321167, 0));
    zone3.initializeZone(
        new Tuple2d(13.69521713256836, 4.061668872833252),
        new Tuple2d(16.51043891906738, 8.078774452209473));
    zone4.initializeZone(
        new Tuple2d(13.69521713256836, 4.061668872833252), new Tuple2d(16.564579010009766, 0));
  }

  public Zone getZone1() {
    return zone1;
  }

  public Zone getZone2() {
    return zone2;
  }

  public Zone getZone3() {
    return zone3;
  }

  public Zone getZone4() {
    return zone4;
  }

  public ZoneId getCurrentZone() {
    if (zone1.isInZoneForAlliance()) {
      return ZoneId.ZONE_1;
    }
    if (zone2.isInZoneForAlliance()) {
      return ZoneId.ZONE_2;
    }
    if (zone3.isInZoneForAlliance()) {
      return ZoneId.ZONE_3;
    }
    if (zone4.isInZoneForAlliance()) {
      return ZoneId.ZONE_4;
    }
    return ZoneId.NONE;
  }

  public Command zoneBasedAim() {
    DoubleSupplier allianceY = () -> AllianceFlipUtil.applyY(drive.getPose().getY());

    return new SelectCommand<>(
        Map.ofEntries(
            Map.entry(ZoneId.ZONE_1, aimToHub()),
            Map.entry(
                ZoneId.ZONE_2,
                new ConditionalCommand(
                    new RotateToOrientation(drive, () -> AllianceFlipUtil.apply(BBumpClosePose)),
                    new RotateToOrientation(
                        drive,
                        () -> AllianceFlipUtil.apply(AllianceFlipUtil.reflectY(BBumpClosePose))),
                    () -> allianceY.getAsDouble() > FieldConstants.fieldWidth / 2)),
            Map.entry(
                ZoneId.ZONE_3,
                new RotateToOrientation(drive, () -> AllianceFlipUtil.apply(BBumpFarPose))),
            Map.entry(
                ZoneId.ZONE_4,
                new RotateToOrientation(
                    drive, () -> AllianceFlipUtil.apply(AllianceFlipUtil.reflectY(BBumpFarPose))))),
        this::getCurrentZone);
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
    Logger.recordOutput("Orchestrator/Pose", drive.getPose());
    Logger.recordOutput("Orchestrator/CurrentZone", getCurrentZone().name());

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

  public Command feedUp() {
    return Commands.repeatingSequence(
            Commands.parallel(magicCarpet.run(), indexer.run())
                .until(() -> !RobotState.getInstance().shooterAtSpeed))
        .onlyIf(() -> shooter.getSetpoint() > 0)
        .onlyIf(() -> RobotState.getInstance().shooterAtSpeed)
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
}
