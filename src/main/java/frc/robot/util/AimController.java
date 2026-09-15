package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.shooter.ShooterConstants.kShooterOffsetFromRobotCenter;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.lib.utils.AllianceFlipUtil;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.Hub;
import frc.robot.commands.auto.DriveToPose;
import frc.robot.commands.auto.RotateToOrientation;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.Zone.Tuple2d;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * AimController encapsulates all drive-aim and zone-based shooting logic that was previously in
 * Superstructure. It is a plain Java class (not a SubsystemBase) — it holds no hardware
 * requirements and produces Commands that require Drive and Shooter internally.
 *
 * <p>Zone geometry, shooter lead compensation, and distance-based setpoint calculations all live
 * here.
 */
public class AimController {

  // ── Zone poses (bump reference points) ───────────────────────────────────

  private final Pose2d BBumpClosePose =
      new Pose2d(4.415811061859131, 5.599213600158691, new Rotation2d());
  private final Pose2d BBumpFarPose =
      new Pose2d(11.334761619567871, 5.599213600158691, new Rotation2d());

  private static final double FRONT_HUB_OFFSET = Units.inchesToMeters(70.0);

  // ── Zone IDs ─────────────────────────────────────────────────────────────

  public enum ZoneId {
    NONE,
    ZONE_1,
    ZONE_2,
    ZONE_3,
    ZONE_4
  }

  // ── Dependencies ──────────────────────────────────────────────────────────

  private final Drive drive;
  private final Shooter shooter;
  private final ShooterLeadCompensator shooterLeadCompensator;

  // ── Zones ─────────────────────────────────────────────────────────────────

  private final Zone zone1;
  private final Zone zone2;
  private final Zone zone3;
  private final Zone zone4;

  // ── Lead compensator output filters ──────────────────────────────────────

  private final LinearFilter targetXFilter = LinearFilter.singlePoleIIR(0.06, 0.02);
  private final LinearFilter targetYFilter = LinearFilter.singlePoleIIR(0.06, 0.02);

  // ── Constructor ───────────────────────────────────────────────────────────

  /**
   * Creates an AimController.
   *
   * @param drive The drive subsystem (used for pose / chassis speeds)
   * @param shooter The shooter subsystem (used for lead-compensation air-time lookup)
   */
  public AimController(Drive drive, Shooter shooter) {
    this.drive = drive;
    this.shooter = shooter;
    this.shooterLeadCompensator = new ShooterLeadCompensator(drive, shooter);

    zone1 = new Zone(drive::getPose);
    zone2 = new Zone(drive::getPose);
    zone3 = new Zone(drive::getPose);
    zone4 = new Zone(drive::getPose);
    zone1.initializeZone(new Tuple2d(0.0, 0.0), new Tuple2d(3.993527889251709, 8.100430488586426));
    zone2.initializeZone(
        new Tuple2d(4.307533264160156, 8.04629135131836), new Tuple2d(11.9086332321167, 0));
    zone3.initializeZone(
        new Tuple2d(13.69521713256836, 4.061668872833252),
        new Tuple2d(16.51043891906738, 8.078774452209473));
    zone4.initializeZone(
        new Tuple2d(13.69521713256836, 4.061668872833252), new Tuple2d(16.564579010009766, 0));
  }

  // ── Zone helpers ──────────────────────────────────────────────────────────

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

  /** Returns the ZoneId the robot is currently in. */
  public ZoneId getCurrentZone() {
    if (zone1.isInZoneForAlliance()) return ZoneId.ZONE_1;
    if (zone2.isInZoneForAlliance()) return ZoneId.ZONE_2;
    if (zone3.isInZoneForAlliance()) return ZoneId.ZONE_3;
    if (zone4.isInZoneForAlliance()) return ZoneId.ZONE_4;
    return ZoneId.NONE;
  }

  // ── Periodic logging ──────────────────────────────────────────────────────

  /** Call from RobotContainer.robotPeriodic() to publish debug values. */
  public void periodic() {
    Logger.recordOutput("AimController/Pose", drive.getPose());
    Logger.recordOutput("AimController/CurrentZone", getCurrentZone().name());

    var swd =
        shooterLeadCompensator.shootWhileDriving(
            AllianceFlipUtil.apply(Hub.innerCenterPoint.toTranslation2d()));
    Logger.recordOutput(
        "AimController/Target",
        new Pose2d(swd.target().getX(), swd.target().getY(), Rotation2d.fromDegrees(0)));
    Logger.recordOutput("AimController/DistanceToHub", swd.distance());
    Logger.recordOutput("AimController/ShooterPosition", shooterLeadCompensator.shooterPose());
  }

  // ── Distance / setpoint helpers ───────────────────────────────────────────

  /** Returns a supplier for the straight-line distance from the shooter to the hub inner center. */
  public Supplier<Distance> getHubDistance() {
    return () ->
        Meters.of(
            AllianceFlipUtil.apply(Hub.innerCenterPoint.toTranslation2d())
                .getDistance(
                    drive.getPose().transformBy(kShooterOffsetFromRobotCenter).getTranslation()));
  }

  /** Returns the shoot-while-driving adjusted target pose (filtered). */
  public Pose2d getShootWhileDrivingResultPose() {
    var swd =
        shooterLeadCompensator.shootWhileDriving(
            AllianceFlipUtil.apply(Hub.innerCenterPoint.toTranslation2d()));
    return new Pose2d(
        targetXFilter.calculate(swd.target().getX()),
        targetYFilter.calculate(swd.target().getY()),
        Rotation2d.fromDegrees(0));
  }

  /** Returns a supplier for the shoot-while-driving compensated distance. */
  public Supplier<Distance> getShootWhileDrivingResultDistance() {
    return () -> {
      var swd =
          shooterLeadCompensator.shootWhileDriving(
              AllianceFlipUtil.apply(Hub.innerCenterPoint.toTranslation2d()));
      return Meters.of(swd.distance());
    };
  }

  // ── Aim commands ──────────────────────────────────────────────────────────

  /**
   * Rotates the robot to face the hub inner center (requires Drive).
   *
   * @return A command that keeps the robot aimed at the hub
   */
  public Command aimToHub() {
    return new DriveToPose(
        drive,
        () ->
            new Pose2d(
                drive.getPose().getX(),
                drive.getPose().getY(),
                AllianceFlipUtil.apply(Hub.innerCenterPoint)
                    .toTranslation2d()
                    .minus(
                        drive.getPose().transformBy(kShooterOffsetFromRobotCenter).getTranslation())
                    .getAngle()));
  }

  /**
   * Pathfinds to the hub approach pose (requires Drive).
   *
   * @return A command that drives to the hub approach position
   */
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

  /**
   * Zone-based aim: picks the correct RotateToOrientation target based on the robot's current zone.
   *
   * @return A SelectCommand that delegates to the correct aim command
   */
  public Command zoneBasedAim() {
    DoubleSupplier allianceY = () -> AllianceFlipUtil.applyY(drive.getPose().getY());
    return Commands.select(
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

  /**
   * Zone-based shooter spin-up: picks the correct distance setpoint based on the robot's current
   * zone.
   *
   * @return A select command that delegates to the correct spinUpDistance command
   */
  public Command zoneBasedShooter() {
    DoubleSupplier allianceY = () -> AllianceFlipUtil.applyY(drive.getPose().getY());
    return Commands.select(
        Map.ofEntries(
            Map.entry(ZoneId.ZONE_1, shooter.spinUpDistance(getHubDistance())),
            Map.entry(
                ZoneId.ZONE_2,
                new ConditionalCommand(
                    shooter.spinUpDistance(
                        () ->
                            Meters.of(
                                AllianceFlipUtil.apply(BBumpClosePose)
                                    .getTranslation()
                                    .getDistance(drive.getPose().getTranslation()))),
                    shooter.spinUpDistance(
                        () ->
                            Meters.of(
                                AllianceFlipUtil.apply(AllianceFlipUtil.reflectY(BBumpClosePose))
                                    .getTranslation()
                                    .getDistance(drive.getPose().getTranslation()))),
                    () -> allianceY.getAsDouble() > FieldConstants.fieldWidth / 2)),
            Map.entry(
                ZoneId.ZONE_3,
                shooter.spinUpDistance(
                    () ->
                        Meters.of(
                            AllianceFlipUtil.apply(BBumpFarPose)
                                .getTranslation()
                                .getDistance(drive.getPose().getTranslation())))),
            Map.entry(
                ZoneId.ZONE_4,
                shooter.spinUpDistance(
                    () ->
                        Meters.of(
                            AllianceFlipUtil.apply(AllianceFlipUtil.reflectY(BBumpFarPose))
                                .getTranslation()
                                .getDistance(drive.getPose().getTranslation()))))),
        this::getCurrentZone);
  }

  /**
   * Spins up shooter at hub close distance (fixed RPM).
   *
   * @return A command that sets the shooter to hub RPM
   */
  public Command spinUpHub() {
    return shooter.spinUpRpm(frc.robot.subsystems.shooter.ShooterConstants.CLOSE_HUB_SHOOTER_RPM);
  }

  /**
   * Spins up shooter to a specific RPM.
   *
   * @param rpm Target RPM
   * @return A command that sets the shooter to the given RPM
   */
  public Command spinUpRpm(double rpm) {
    return shooter.spinUpRpm(rpm);
  }

  /**
   * Spins up shooter to the distance-based setpoint.
   *
   * @param targetDistance Supplier returning the target distance
   * @return A command that sets the shooter to the distance-based setpoint
   */
  public Command spinUpDistance(Supplier<Distance> targetDistance) {
    return shooter.spinUpDistance(targetDistance);
  }

  /**
   * Spin and drive at the angle calculated by shoot-while-driving lead compensation.
   *
   * @param leftXSupplier Driver left-X axis
   * @param leftYSupplier Driver left-Y axis
   * @return A parallel command combining aim drive and shooter spin-up
   */
  public Command driveShootAtAngle(DoubleSupplier leftXSupplier, DoubleSupplier leftYSupplier) {
    return Commands.parallel(
        shooter.setTargetVelocity(shooter.calculateSetpoint(getShootWhileDrivingResultDistance())),
        DriveCommands.joystickDriveAtAngle(
            drive,
            leftXSupplier,
            leftYSupplier,
            () ->
                getShootWhileDrivingResultPose()
                    .getTranslation()
                    .minus(drive.getPose().getTranslation())
                    .getAngle()));
  }
}
