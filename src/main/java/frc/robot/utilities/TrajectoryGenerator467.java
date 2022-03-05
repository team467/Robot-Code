package frc.robot.utilities;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.RobotConstants;
import java.util.List;

public class TrajectoryGenerator467 {
  public static Trajectory generateTrajectory(
      Pose2d initalPose, List<Translation2d> interiorWaypoints, Pose2d endingPose, boolean reversed) {

    DifferentialDriveVoltageConstraint autDriveVoltageConstraint = new DifferentialDriveVoltageConstraint(
        RobotConstants.get().driveDriveFF().getFeedforward(),
        RobotConstants.get().driveKinematics(),
        10);

    // Add kinematics to ensure max speed is actually obeyed
    // Apply the voltage constraint
    TrajectoryConfig config = new TrajectoryConfig(
        RobotConstants.get().driveAutoMaxVelocity(),
        RobotConstants.get().driveAutoMaxAcceleration())
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(RobotConstants.get().driveKinematics())
        // Apply the voltage constraint
        .addConstraint(autDriveVoltageConstraint)
        .setReversed(reversed);


    return TrajectoryGenerator.generateTrajectory(
        initalPose,
        interiorWaypoints,
        endingPose,
        config
    );
  }
}
