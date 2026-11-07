package frc.robot.commands.auto;

import frc.robot.subsystems.drive.Drive;
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
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController( // TODO: Tune this
          2.5, 0, .01, new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(720.0)));
  private Rotation2d targetPose;

  public RotateToOrientation() {
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public Command get(Drive drive, Supplier<Rotation2d> rotationSupplier) {
    return Command.noRequirements(coro -> {
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
        ).named("RotateToOrientation Run")
        .andThen(Command.noRequirements(_ -> drive.stop()).named("Stop Drive"))
        .named("RotateToOrientation");
  }
}
