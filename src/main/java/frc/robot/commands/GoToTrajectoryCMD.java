package frc.robot.commands;

import frc.robot.utilities.TrajectoryGenerator467;
import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gyro;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class GoToTrajectoryCMD extends CommandBase {
    private final Drivetrain drivetrain;
    private final Gyro gyro;
    private final DifferentialDriveOdometry diffDriveOdometry;
    private final RamseteCommand command;
    private final Trajectory trajectory;

    public GoToTrajectoryCMD(Drivetrain drivetrain, Gyro gyro, Pose2d initalPose, List<Translation2d> interiorWaypoints, Pose2d endingPose, boolean reversed) {
        this(drivetrain, gyro, TrajectoryGenerator467.generateTrajectory(initalPose, interiorWaypoints, endingPose, reversed));
    }

    public GoToTrajectoryCMD(Drivetrain drivetrain, Gyro gyro, Trajectory trajectory) {
        this.drivetrain = drivetrain;
        this.gyro = gyro;
        this.trajectory = trajectory;
        this.diffDriveOdometry = new DifferentialDriveOdometry(gyro.getRotation2d());

        this.command = new RamseteCommand(
                trajectory,
                diffDriveOdometry::getPoseMeters,
                RobotConstants.get().driveRamsete().getController(),
                RobotConstants.get().driveDriveFF().getFeedforward(),
                RobotConstants.get().driveKinematics(),
                drivetrain::getWheelSpeeds,
                RobotConstants.get().driveDriveVelocityPID().getPIDController(),
                RobotConstants.get().driveDriveVelocityPID().getPIDController(),
                // RamseteCommand passes volts to the callback
                drivetrain::tankDriveVolts,
                drivetrain
        );

        addRequirements(gyro);
    }

    @Override
    public void initialize() {
        drivetrain.resetPositions();
        diffDriveOdometry.resetPosition(trajectory.getInitialPose(), gyro.getRotation2d());

        command.initialize();
    }

    @Override
    public void execute() {
        diffDriveOdometry.update(gyro.getRotation2d(), drivetrain.getLeftPosition(), drivetrain.getRightPosition());
        command.execute();
    }

    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);
        drivetrain.tankDriveVolts(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return command.isFinished();
    }
}
