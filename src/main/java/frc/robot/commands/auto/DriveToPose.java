package frc.robot.commands.auto;

import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.MetersPerSecondPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RadiansPerSecondPerSecond;

import frc.lib.utils.GeomUtils;
import frc.lib.utils.Tunable;
import frc.robot.subsystems.swerve.Drive;
import java.util.Optional;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities; // Removed or renamed
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.LinearAcceleration;
import org.wpilib.units.measure.LinearVelocity;

/**
 * Command to drive the robot to a specified pose in a straight path. This is useful for very simple
 * autos or for alignment.
 */
public class DriveToPose {

  @Tunable
  private static final double driveKp = 2.5;
  @Tunable
  private static final double driveKd = 0.0;
  @Tunable
  private static final double thetaKp = 3.0;
  @Tunable
  private static final double thetaKd = 0.01;
  @Tunable
  private static final LinearVelocity driveMaxVelocity = MetersPerSecond.of(
      Units.inchesToMeters(85));
  @Tunable
  private static final LinearAcceleration driveMaxAcceleration = MetersPerSecondPerSecond.of(
      Units.inchesToMeters(450));
  @Tunable
  private static final AngularVelocity thetaMaxVelocity = RadiansPerSecond.of(
      Units.degreesToRadians(360));
  @Tunable
  private static final AngularAcceleration thetaMaxAcceleration = RadiansPerSecondPerSecond.of(
      Units.degreesToRadians(720));
  @Tunable
  private static final double driveTolerance = 0.01;
  @Tunable
  private static final double thetaTolerance = Units.degreesToRadians(0.2);

  private final ProfiledPIDController driveController =
      new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(1.0, 1.0));
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(1.0, 1.0));

  public Command get(Drive drive, Supplier<Pose2d> poseSupplier) {
    return drive.run(coro -> {
          thetaController.enableContinuousInput(-Math.PI, Math.PI);

          updateControllers();

          var currentPose = drive.getPose();
          driveController.reset(
              currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation()));
          thetaController.reset(currentPose.getRotation().getRadians());

          while (!(driveController.atGoal() && thetaController.atGoal())) {
            // Update from tunable numbers
            updateControllers();

            // Get current and target pose
            currentPose = drive.getPose();
            var targetPose = poseSupplier.get();

            // Calculate drive speed
            double driveErrorAbs = currentPose.getTranslation()
                .getDistance(targetPose.getTranslation());
            double driveVelocityScalar = driveController.calculate(driveErrorAbs, 0.0);
            if (driveController.atGoal()) {
              driveVelocityScalar = 0.0;
            }

            // Calculate theta speed
            double thetaVelocity =
                thetaController.calculate(
                    currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
            if (thetaController.atGoal()) {
              thetaVelocity = 0.0;
            }

            // Command speeds
            var angleToTarget = currentPose.getTranslation().minus(targetPose.getTranslation())
                .getAngle();
            if (angleToTarget.isEmpty()) {
              angleToTarget = Optional.of(Rotation2d.ZERO);
            }
            var driveVelocity =
                new Pose2d(
                    new Translation2d(),
                    angleToTarget.get())
                    .transformBy(GeomUtils.transformFromTranslation(driveVelocityScalar, 0.0))
                    .getTranslation();
            drive.runVelocity(
                new ChassisVelocities(
                    driveVelocity.getX(), driveVelocity.getY(),
                    thetaVelocity).toRobotRelative(currentPose.getRotation()));

            coro.yield();
          }
        }
    ).whenCanceled(drive::stop).named("DriveToPose");
  }

  private synchronized void updateControllers() {
    driveController.setP(driveKp);
    driveController.setD(driveKd);

    driveController.setConstraints(
        new TrapezoidProfile.Constraints(driveMaxVelocity.in(MetersPerSecond),
            driveMaxAcceleration.in(MetersPerSecondPerSecond)));
    driveController.setTolerance(driveTolerance);
    thetaController.setP(thetaKp);
    thetaController.setD(thetaKd);
    thetaController.setConstraints(
        new TrapezoidProfile.Constraints(thetaMaxVelocity.in(RadiansPerSecond),
            thetaMaxAcceleration.in(RadiansPerSecondPerSecond)));
    thetaController.setTolerance(thetaTolerance);
  }
}
