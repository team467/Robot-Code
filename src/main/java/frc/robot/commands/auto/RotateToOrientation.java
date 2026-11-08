package frc.robot.commands.auto;

import frc.robot.subsystems.swerve.Drive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities; // Removed or renamed
import org.wpilib.math.trajectory.TrapezoidProfile.Constraints;
import org.wpilib.math.util.Units;

public class RotateToOrientation {

  private static final double THETA_TOLERANCE = Units.degreesToRadians(1.0);
  private static final double DRIVE_TOLERANCE = 0.005;
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController( // TODO: Tune this
          2.5, 0, .01, new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(720.0)));
  private Rotation2d targetPose;

  public RotateToOrientation() {
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public Command get(Drive drive, Supplier<Rotation2d> rotationSupplier) {
    return drive.run(coro -> {
          Pose2d currentPose = drive.getPose();
          targetPose = rotationSupplier.get();
          Logger.recordOutput("RotateToOrientation/TargetPose", targetPose);
          thetaController.enableContinuousInput(-Math.PI, Math.PI);
          thetaController.setTolerance(THETA_TOLERANCE);
          thetaController.reset(currentPose.getRotation().getRadians());

          while (!thetaController.atGoal()) {
            currentPose = drive.getPose();
            targetPose = rotationSupplier.get();

            // Calculate theta speed
            double thetaVelocity =
                thetaController.calculate(currentPose.getRotation().getRadians(),
                    targetPose.getRadians());
            if (thetaController.atGoal()) {
              thetaVelocity = 0.0;
            }

            drive.runVelocity(
                new ChassisVelocities(0, 0, thetaVelocity).toFieldRelative(currentPose.getRotation()));

            coro.yield();
          }
        }
    ).whenCanceled(drive::stop).named("RotateToOrientation");
  }

  public Command get(Drive drive, Pose2d targetPose) {
    return get(drive,
        () -> targetPose.getTranslation().minus(drive.getPose().getTranslation()).getAngle().get());
  }
}
