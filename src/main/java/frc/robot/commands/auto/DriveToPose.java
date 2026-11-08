package frc.robot.commands.auto;

import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.MetersPerSecondPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RadiansPerSecondPerSecond;

import frc.lib.utils.GeomUtils;
import frc.lib.utils.Tunable;
import frc.robot.Constants;
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

  private static final Tunable<Double> driveKp;
  private static final Tunable<Double> driveKd;
  private static final Tunable<Double> thetaKp;
  private static final Tunable<Double> thetaKd;
  private static final Tunable<LinearVelocity> driveMaxVelocity;
  private static final Tunable<LinearAcceleration> driveMaxAcceleration;
  private static final Tunable<AngularVelocity> thetaMaxVelocity;
  private static final Tunable<AngularAcceleration> thetaMaxAcceleration;
  private static final Tunable<Double> driveTolerance;
  private static final Tunable<Double> thetaTolerance;

  static {
    switch (Constants.getRobot()) {
      case ROBOT_2026_COMP:
      default:
        driveKp = new Tunable<>("DriveToPose/DriveKp", 2.5);
        driveKd = new Tunable<>("DriveToPose/DriveKd", 0.0);
        thetaKp = new Tunable<>("DriveToPose/ThetaKp", 3.0);
        thetaKd = new Tunable<>("DriveToPose/ThetaKd", 0.01);
        driveMaxVelocity = new Tunable<>("DriveToPose/DriveMaxVelocity",
            MetersPerSecond.of(Units.inchesToMeters(85)));
        driveMaxAcceleration = new Tunable<>("DriveToPose/DriveMaxAcceleration",
            MetersPerSecondPerSecond.of(Units.inchesToMeters(450)));
        thetaMaxVelocity = new Tunable<>("DriveToPose/ThetaMaxVelocity",
            RadiansPerSecond.of(Units.degreesToRadians(360)));
        thetaMaxAcceleration = new Tunable<>("DriveToPose/ThetaMaxAcceleration",
            RadiansPerSecondPerSecond.of(Units.degreesToRadians(720)));
        driveTolerance = new Tunable<>("DriveToPose/DriveTolerance", 0.01);
        thetaTolerance = new Tunable<>("DriveToPose/ThetaTolerance", Units.degreesToRadians(0.2));
    }
  }

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

  private void updateControllers() {
    driveController.setP(driveKp.get());
    driveController.setD(driveKd.get());

    driveController.setConstraints(
        new TrapezoidProfile.Constraints(driveMaxVelocity.get().in(MetersPerSecond),
            driveMaxAcceleration.get().in(MetersPerSecondPerSecond)));
    driveController.setTolerance(driveTolerance.get());
    thetaController.setP(thetaKp.get());
    thetaController.setD(thetaKd.get());
    thetaController.setConstraints(
        new TrapezoidProfile.Constraints(thetaMaxVelocity.get().in(RadiansPerSecond),
            thetaMaxAcceleration.get().in(RadiansPerSecondPerSecond)));
    thetaController.setTolerance(thetaTolerance.get());
  }
}
