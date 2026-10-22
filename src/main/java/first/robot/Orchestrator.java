package first.robot;

import static first.robot.mechanisms.shooter.ShooterConstants.CLOSE_HUB_SHOOTER_RPM;
import static first.robot.mechanisms.shooter.ShooterConstants.kShooterOffsetFromRobotCenter;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Seconds;

import first.robot.constants.FieldConstants;
import first.robot.constants.FieldConstants.Hub;
import first.robot.commands.auto.DriveToPose;
import first.robot.commands.auto.RotateToOrientation;
import first.robot.constants.IntakeConstants;
import first.robot.mechanisms.Intake.Intake;
import first.robot.mechanisms.indexer.Indexer;
import first.robot.mechanisms.magicCarpet.MagicCarpet;
import first.robot.mechanisms.shooter.Shooter;
import first.robot.mechanisms.swerve.Drive;
import first.robot.util.AllianceFlipUtil;
import first.robot.util.ShooterLeadCompensator;
import first.robot.util.Zone;
import first.robot.util.Zone.Tuple2d;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.wpilib.command3.Command;
import org.wpilib.command3.ParallelGroupBuilder;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.math.filter.LinearFilter;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Distance;

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

  private final double FRONT_HUB_OFFSET = Units.inchesToMeters(70.0);
  private final Drive drive;
  private final Shooter shooter;
  private final MagicCarpet magicCarpet;
  private final Indexer indexer;
  private final Intake intake;
  private final ShooterLeadCompensator shooterLeadCompensator;
  private final CommandGamepad driverController;

  private final LinearFilter targetXFilter = LinearFilter.singlePoleIIR(0.06, 0.02);
  private final LinearFilter targetYFilter = LinearFilter.singlePoleIIR(0.06, 0.02);

  private final LoggedNetworkNumber testRpm =
      new LoggedNetworkNumber("/Tuning/Shooter/TestRPM", CLOSE_HUB_SHOOTER_RPM);

  private final Zone zone1;
  private final Zone zone2;
  private final Zone zone3;
  private final Zone zone4;

  public Orchestrator(
      Drive drive,
      MagicCarpet magicCarpet,
      Shooter shooter,
      Indexer indexer,
      Intake intake,
      CommandGamepad driverController) {
    this.drive = drive;
    this.magicCarpet = magicCarpet;
    this.shooter = shooter;
    this.indexer = indexer;
    this.intake = intake;
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

    Scheduler.getDefault().addPeriodic(this::orchestratorPeriodic);
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

  private boolean onUpperHalf() {
    return AllianceFlipUtil.applyY(drive.getPose().getY()) > FieldConstants.fieldWidth / 2;
  }

  private Translation2d zoneTarget(ZoneId zone) {
    return switch (zone) {
      case ZONE_1 -> AllianceFlipUtil.apply(Hub.innerCenterPoint.toTranslation2d());
      case ZONE_2 ->
          onUpperHalf()
              ? AllianceFlipUtil.apply(BBumpClosePose).getTranslation()
              : AllianceFlipUtil.apply(AllianceFlipUtil.reflectY(BBumpClosePose)).getTranslation();
      case ZONE_3 -> AllianceFlipUtil.apply(BBumpFarPose).getTranslation();
      case ZONE_4 -> AllianceFlipUtil.apply(AllianceFlipUtil.reflectY(BBumpFarPose)).getTranslation();
      case NONE -> null;
    };
  }

  public Command zoneBasedShooter() {
    return Command.noRequirements(
            coroutine -> {
              ZoneId zone = getCurrentZone();
              if (zone == ZoneId.ZONE_1) {
                coroutine.await(spinUpShooterDistance(getHubDistance()));
              } else if (zone != ZoneId.NONE) {
                Translation2d target = zoneTarget(zone);
                coroutine.await(
                    spinUpShooterDistance(
                        () -> Meters.of(target.getDistance(drive.getPose().getTranslation()))));
              }
            })
        .named("zoneBasedShooter");
  }

  public Command zoneBasedAim() {
    return Command.noRequirements(
            coroutine -> {
              Command aim =
                  switch (getCurrentZone()) {
                    case ZONE_1 -> aimToHub();
                    case ZONE_2 ->
                        onUpperHalf()
                            ? new RotateToOrientation(
                                drive, () -> AllianceFlipUtil.apply(BBumpClosePose))
                            : new RotateToOrientation(
                                drive,
                                () ->
                                    AllianceFlipUtil.apply(
                                        AllianceFlipUtil.reflectY(BBumpClosePose)));
                    case ZONE_3 ->
                        new RotateToOrientation(drive, () -> AllianceFlipUtil.apply(BBumpFarPose));
                    case ZONE_4 ->
                        new RotateToOrientation(
                            drive,
                            () -> AllianceFlipUtil.apply(AllianceFlipUtil.reflectY(BBumpFarPose)));
                    case NONE -> null;
                  };
              if (aim != null) {
                coroutine.await(aim);
              }
            })
        .named("zoneBasedAim");
  }

  public Pose2d getShootWhileDrivingResultPose() {
    var shootWhileDrivingResult =
        shooterLeadCompensator.shootWhileDriving(
            AllianceFlipUtil.apply(Hub.innerCenterPoint.toTranslation2d()));
    return new Pose2d(
        targetXFilter.calculate(shootWhileDrivingResult.target().getX()),
        targetYFilter.calculate(shootWhileDrivingResult.target().getY()),
        Rotation2d.ZERO);
  }

  public Supplier<Distance> getShootWhileDrivingResultDistance() {
    return () -> {
      var shootWhileDrivingResult =
          shooterLeadCompensator.shootWhileDriving(
              AllianceFlipUtil.apply(Hub.innerCenterPoint.toTranslation2d()));
      return Meters.of(shootWhileDrivingResult.distance());
    };
  }

  public Supplier<Distance> getHubDistance() {
    return () ->
        Meters.of(
            AllianceFlipUtil.apply(Hub.innerCenterPoint.toTranslation2d())
                .getDistance(
                    drive.getPose().transformBy(kShooterOffsetFromRobotCenter).getTranslation()));
  }

  private void orchestratorPeriodic() {
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
            Rotation2d.ZERO));
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
                          new Transform2d(FRONT_HUB_OFFSET, 0.0, Rotation2d.ZERO)));
              Rotation2d sameAsAlignAndShootHeading =
                  headingTo(
                      AllianceFlipUtil.apply(Hub.blueCenter), hubApproachPose.getTranslation());
              return new Pose2d(hubApproachPose.getTranslation(), sameAsAlignAndShootHeading);
            });
  }

  private static final double FEED_SPINUP_TIMEOUT = 1.0;

  public Command feedUp() {
    return Command.noRequirements(
            coroutine -> {
              coroutine.waitUntil(shooter::isReadyToShoot, Seconds.of(FEED_SPINUP_TIMEOUT));
              if (shooterSpinning()) {
                coroutine.await(
                    indexer.run().until(() -> !shooterSpinning()).named("feedUp.run"));
              }
            })
        .named("feedUp");
  }

  public Command shoot() {
    return Command.noRequirements(
            coroutine -> {
              if (!shooterSpinning()) {
                return;
              }
              coroutine.await(
                  new ParallelGroupBuilder()
                      .requiring(feedUp())
                      .optional(intake.slowlyBringInThenShake())
                      .named("shoot.feedAndCollapse"));
            })
        .named("shoot");
  }

  private boolean shooterSpinning() {
    return shooter.getSetpoint().gt(RadiansPerSecond.of(0));
  }

  public Command spinUpShooterTest() {
    return shooter.setTargetVelocity(() -> RPM.of(testRpm.get()));
  }

  public Command spinUpShooterDistance(Supplier<Distance> targetDistance) {
    return shooter.setTargetVelocity(shooter.calculateSetpoint(targetDistance));
  }

  public Command spinUpShooterHub() {
    return shooter.setTargetVelocity(RPM.of(CLOSE_HUB_SHOOTER_RPM));
  }

  public Command spinUpShooter(double velocityRPM) {
    return shooter.setTargetVelocity(RPM.of(velocityRPM));
  }

  public Command shootWhileRetractingIntake(Command shooterCommand) {
    return Command.parallel(
            intake.extendToAngleAndIntake(IntakeConstants.COLLAPSE_POS), shooterCommand, feedUp())
        .named("shootWhileRetractingIntake");
  }

  public Command shootWhileRetractingIntakeHub() {
    return shootWhileRetractingIntake(spinUpShooterHub());
  }

  public Command shootWhileRetractingIntakeDistance(Supplier<Distance> targetDistance) {
    return shootWhileRetractingIntake(spinUpShooterDistance(targetDistance));
  }

  public Command driveShootAtAngle() {
    return Command.parallel(
            spinUpShooterWhileDriving(),
            drive.joystickDriveAtAngle(
                driverController,
                () ->
                    headingTo(
                        getShootWhileDrivingResultPose().getTranslation(),
                        drive.getPose().getTranslation())))
        .named("driveShootAtAngle");
  }

  public Command spinUpShooterWhileDriving() {
    return shooter.setTargetVelocity(
        shooter.calculateSetpoint(this.getShootWhileDrivingResultDistance()));
  }

  public Command aimToHub() {
    return new DriveToPose(
        drive,
        () ->
            new Pose2d(
                drive.getPose().getX(),
                drive.getPose().getY(),
                headingTo(
                    AllianceFlipUtil.apply(Hub.innerCenterPoint.toTranslation2d()),
                    drive.getPose().transformBy(kShooterOffsetFromRobotCenter).getTranslation())));
  }

  private Rotation2d headingTo(Translation2d target, Translation2d from) {
    return target.minus(from).getAngle().orElse(drive.getRotation());
  }

  public Command stopShootingAuto() {
    return Command.parallel(indexer.stop(), shooter.stop()).named("stopShootingAuto");
  }
}
