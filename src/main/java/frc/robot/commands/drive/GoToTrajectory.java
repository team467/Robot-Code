package frc.robot.commands.drive;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.holonomictrajectory.CustomTrajectoryGenerator;
import frc.lib.holonomictrajectory.SwerveControllerCommand;
import frc.lib.holonomictrajectory.Waypoint;
import frc.robot.RobotConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.List;

public class GoToTrajectory extends CommandBase {

  private final SwerveControllerCommand command;

  private final CustomTrajectoryGenerator customGenerator = new CustomTrajectoryGenerator();

  public GoToTrajectory(Drive drive, List<Waypoint> waypoints) {
    this(drive, waypoints, 0, 0, List.of());
  }

  public GoToTrajectory(
      Drive drive, List<Waypoint> waypoints, double startVelocity, double endVelocity) {
    this(drive, waypoints, startVelocity, endVelocity, List.of());
  }

  public GoToTrajectory(
      Drive drive,
      List<Waypoint> waypoints,
      double startVelocity,
      double endVelocity,
      List<TrajectoryConstraint> constraints) {
    addRequirements(drive);

    TrajectoryConfig config =
        new TrajectoryConfig(
                RobotConstants.get().chassisDriveMaxVelocity(),
                RobotConstants.get().chassisDriveMaxAcceleration())
            .setKinematics(RobotConstants.get().kinematics())
            .setStartVelocity(startVelocity)
            .setEndVelocity(endVelocity)
            .addConstraints(constraints);

    try {
      customGenerator.generate(config, waypoints);
    } catch (TrajectoryGenerationException e) {
      DriverStation.reportError("Failed to generate trajectory.", e.getStackTrace());
    }

    command =
        new SwerveControllerCommand(
            customGenerator.getDriveTrajectory(),
            customGenerator.getHolonomicRotationSequence(),
            drive::getPose,
            RobotConstants.get().chassisDriveFB().getPIDController(),
            RobotConstants.get().chassisDriveFB().getPIDController(),
            RobotConstants.get()
                .chassisTurnFB()
                .getProfiledPIDController(
                    new Constraints(
                        RobotConstants.get().chassisTurnMaxVelocity(),
                        RobotConstants.get().chassisTurnMaxAcceleration())),
            drive::runVelocity,
            drive);
  }

  @Override
  public void initialize() {
    command.initialize();
  }

  @Override
  public void execute() {
    command.execute();
  }

  @Override
  public void end(boolean interrupted) {
    command.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return command.isFinished();
  }
}
