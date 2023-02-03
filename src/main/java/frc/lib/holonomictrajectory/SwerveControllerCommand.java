package frc.lib.holonomictrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.utils.AllianceFlipUtil;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * A command that uses two PID controllers ({@link PIDController}) and a ProfiledPIDController
 * ({@link ProfiledPIDController}) to follow a trajectory {@link Trajectory} and rotation sequence
 * {@link RotationSequence} with a swerve drive.
 *
 * <p>This command outputs the desired Chassis Speeds ({@link ChassisSpeeds}). The desired wheel and
 * module rotation velocities should be taken from a kinematics conversion and used in velocity
 * PIDs.
 */
public class SwerveControllerCommand extends CommandBase {

  private final Trajectory driveTrajectory;
  private final RotationSequence holonomicRotationSequence;
  private final PIDController xController;
  private final PIDController yController;
  private final ProfiledPIDController thetaController;

  private final CustomHolonomicDriveController controller;

  private final Supplier<Pose2d> pose;
  private final Consumer<ChassisSpeeds> output;
  private final Timer timer = new Timer();

  /**
   * Constructs a new SwerveControllerCommand that when executed will follow the provided
   * trajectory. This command will not return output voltages but rather a chassis speed from the
   * position controllers which need to be put into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the outputVolts to zero upon completion of the path,
   * this is left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * @param driveTrajectory The driving trajectory to follow.
   * @param holonomicRotationSequence The holonomic turning sequence to follow.
   * @param pose A function that supplies the robot pose - use one of the odometry classes to
   *     provide this.
   * @param xController The Trajectory Tracker PID controller for the robot's x position.
   * @param yController The Trajectory Tracker PID controller for the robot's y position.
   * @param thetaController The Trajectory Tracker PID controller for angle for the robot.
   * @param output The raw output module states from the position controllers.
   * @param requirements The subsystems to require.
   */
  public SwerveControllerCommand(
      Trajectory driveTrajectory,
      RotationSequence holonomicRotationSequence,
      Supplier<Pose2d> pose,
      PIDController xController,
      PIDController yController,
      ProfiledPIDController thetaController,
      Consumer<ChassisSpeeds> output,
      Subsystem... requirements) {
    this.driveTrajectory = driveTrajectory;
    this.holonomicRotationSequence = holonomicRotationSequence;
    this.pose = pose;
    this.xController = xController;
    this.yController = yController;
    this.thetaController = thetaController;
    this.controller =
        new CustomHolonomicDriveController(
            this.xController, this.yController, this.thetaController);
    this.output = output;

    addRequirements(requirements);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController.reset();
    yController.reset();
    thetaController.reset(pose.get().getRotation().getRadians());

    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Trajectory.State driveState = AllianceFlipUtil.apply(driveTrajectory.sample(timer.get()));
    RotationSequence.State holonomicRotationState = AllianceFlipUtil.apply(holonomicRotationSequence.sample(timer.get()));

    ChassisSpeeds nextDriveState =
        controller.calculate(pose.get(), driveState, holonomicRotationState);
    output.accept(nextDriveState);

    Logger.getInstance()
        .recordOutput(
            "Odometry/ProfileSetpoint",
            new Pose2d(
                driveState.poseMeters.getX(),
                driveState.poseMeters.getY(),
                holonomicRotationState.position));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    output.accept(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(driveTrajectory.getTotalTimeSeconds());
  }
}
