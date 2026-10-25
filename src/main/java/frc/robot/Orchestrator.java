package frc.robot;

import static frc.robot.subsystems.shooter.ShooterConstants.CLOSE_HUB_SHOOTER_RPM;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Minute;
import static org.wpilib.units.Units.Rotations;

import frc.lib.utils.AllianceFlipUtil;
import frc.robot.FieldConstants.Hub;
import frc.robot.commands.auto.DriveToPose;
import frc.robot.commands.auto.RotateToOrientation;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.ShooterLeadCompensator;
import frc.robot.util.Zone;
import frc.robot.util.Zone.Tuple2d;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.ConditionalCommand;
import org.wpilib.command2.SelectCommand;
import org.wpilib.command3.Command;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Distance;

public class Orchestrator {

  private static final double ANGLE_ALPHA =
      0.15; // tune: lower = smoother, higher = more responsive
  private final Pose2d BBumpClosePose =
      new Pose2d(4.415811061859131, 5.599213600158691, new Rotation2d());
  private final Pose2d BBumpFarPose =
      new Pose2d(11.334761619567871, 5.599213600158691, new Rotation2d());
  private final double FRONT_HUB_OFFSET = Units.inchesToMeters(58.0);
  private final Drive drive;
  private final Shooter shooter;
  private final MagicCarpet magicCarpet;
  private final Indexer indexer;
  private final Intake intake;
  private final ShooterLeadCompensator shooterLeadCompensator;
  private final CommandGamepad driverController;
  // Filtering for lead compensator outputs
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
      CommandGamepad driverController) {
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

  public Command zoneBasedShooter() {
    DoubleSupplier allianceY = () -> AllianceFlipUtil.applyY(drive.getPose().getY());

    return new SelectCommand<>(
        Map.ofEntries(
            Map.entry(ZoneId.ZONE_1, spinUpShooterDistance(getHubDistance())),
            Map.entry(
                ZoneId.ZONE_2,
                new ConditionalCommand(
                    spinUpShooterDistance(
                        () ->
                            Meters.of(
                                AllianceFlipUtil.apply(BBumpClosePose)
                                    .getTranslation()
                                    .getDistance((drive.getPose().getTranslation())))),
                    spinUpShooterDistance(
                        () ->
                            Meters.of(
                                AllianceFlipUtil.apply(AllianceFlipUtil.reflectY(BBumpClosePose))
                                    .getTranslation()
                                    .getDistance(drive.getPose().getTranslation()))),
                    () -> allianceY.getAsDouble() > FieldConstants.fieldWidth / 2)),
            Map.entry(
                ZoneId.ZONE_3,
                spinUpShooterDistance(
                    () ->
                        Meters.of(
                            AllianceFlipUtil.apply(BBumpFarPose)
                                .getTranslation()
                                .getDistance(drive.getPose().getTranslation())))),
            Map.entry(
                ZoneId.ZONE_4,
                spinUpShooterDistance(
                    () ->
                        Meters.of(
                            AllianceFlipUtil.apply(AllianceFlipUtil.reflectY(BBumpFarPose))
                                .getTranslation()
                                .getDistance(drive.getPose().getTranslation()))))),
        this::getCurrentZone);
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

  public Supplier<Distance> getHubDistance() {
    return () ->
        Meters.of(
            AllianceFlipUtil.apply(Hub.innerCenterPoint.toTranslation2d())
                .getDistance(
                    drive
                        .getPose()
                        .transformBy(ShooterConstants.kShooterOffsetFromRobotCenter)
                        .getTranslation()));
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
    return Command.noRequirements(
        coro -> {
          while (!shooter.atSetpoint()) {
            indexer.run();
            coro.yield();
          }
        }
    ).named("Feed Up");
  }

  public Command spinUpShooterDistance(Supplier<Distance> targetDistance) {
    return shooter.runAtVelocity(shooter.calculateSetpoint(targetDistance).get());
  }

  public Command spinUpShooterHub() {
    return shooter.runAtVelocity(Rotations.per(Minute).of(CLOSE_HUB_SHOOTER_RPM));
  }

  public Command aimToHub() {
    return new DriveToPose(
        drive,
        () ->
            new Pose2d(
                drive.getPose().getX(),
                drive.getPose().getY(),
                AllianceFlipUtil.apply(Hub.blueCenter)
                    .plus(new Translation2d(-0.4, 0))
                    .minus(
                        drive
                            .getPose()
                            .transformBy(ShooterConstants.kShooterOffsetFromRobotCenter)
                            .getTranslation())
                    .getAngle()));
  }

  public Command stopShootingAuto() {
    return Command.noRequirements(
        coro -> {
          coro.fork(indexer.stop());
          coro.fork(shooter.stop());
        }
    ).named("Stop Shooting Auto");
  }

  public enum ZoneId {
    NONE,
    ZONE_1,
    ZONE_2,
    ZONE_3,
    ZONE_4
  }
}
