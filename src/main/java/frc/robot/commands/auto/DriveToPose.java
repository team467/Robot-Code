package frc.robot.commands.auto;

import frc.lib.utils.GeomUtils;
import frc.lib.utils.TunableNumber;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Drive;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities; // Removed or renamed
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.util.Units;

/**
 * Command to drive the robot to a specified pose in a straight path. This is useful for very simple
 * autos or for alignment.
 */
public class DriveToPose {

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
      case ROBOT_2026_COMP:
        driveKp.initDefault(2.5);
        driveKd.initDefault(0.0);
        thetaKp.initDefault(3.0);
        thetaKd.initDefault(0.01);
        driveMaxVelocity.initDefault(Units.inchesToMeters(85));
        driveMaxAcceleration.initDefault(Units.inchesToMeters(450.0));
        thetaMaxVelocity.initDefault(Units.degreesToRadians(360.0));
        thetaMaxAcceleration.initDefault(Units.degreesToRadians(720.0));
        driveTolerance.initDefault(0.01);
        thetaTolerance.initDefault(Units.degreesToRadians(0.2));
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

  public Command get(Drive drive, Supplier<Pose2d> poseSupplier) {
    return drive.run(coro -> {
              final ProfiledPIDController driveController =
                  new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));
              final ProfiledPIDController thetaController =
                  new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0));
              double driveErrorAbs;

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

              while (!(driveController.atGoal() && thetaController.atGoal())) {
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
                currentPose = drive.getPose();
                var targetPose = poseSupplier.get();

                // Calculate drive speed
                driveErrorAbs = currentPose.getTranslation()
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
                var driveVelocity =
                    new Pose2d(
                        new Translation2d(),
                        currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle().get())
                        .transformBy(GeomUtils.transformFromTranslation(driveVelocityScalar, 0.0))
                        .getTranslation();
                drive.runVelocity(
                    new ChassisVelocities(
                        driveVelocity.getX(), driveVelocity.getY(),
                        thetaVelocity).toFieldRelative(currentPose.getRotation()));

                coro.yield();
              }
            }
        ).named("DriveToPose Run")
        .andThen(Command.noRequirements(_ -> drive.stop()).named("Stop Drive"))
        .named("Drive To Pose");
  }
}
