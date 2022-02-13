package frc.robot.commands;

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

public class TurnAngleCMD extends CommandBase {
    private final Drivetrain drivetrain;
    private final Gyro gyro;
    private final DifferentialDriveOdometry diffDriveOdometry;
    private final RamseteCommand command;

    public TurnAngleCMD(Drivetrain drivetrain, Gyro gyro, double angle) {
        this.drivetrain = drivetrain;
        this.gyro = gyro;

        diffDriveOdometry = new DifferentialDriveOdometry(gyro.getRotation2d());

        DifferentialDriveVoltageConstraint autDriveVoltageConstraint = new DifferentialDriveVoltageConstraint(
                RobotConstants.get().driveDriveFF().getFeedforward(),
                RobotConstants.get().driveKinematics(),
                10);

        // Add kinematics to ensure max speed is actually obeyed
        // Apply the voltage constraint
        TrajectoryConfig config = new TrajectoryConfig(RobotConstants.get().driveMaxVelocity(),
                RobotConstants.get().driveMaxAcceleration())
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(RobotConstants.get().driveKinematics())
                // Apply the voltage constraint
                .addConstraint(autDriveVoltageConstraint);


        // Start at the origin facing the +X direction
        // Pass through these two interior waypoints, making an 's' curve path
        // End 3 meters straight ahead of where we started, facing forward
        // Pass config
        // TODO TURN ANGLE
        Trajectory turnTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(
                        new Translation2d(1, 1),
                        new Translation2d(2, -1)
                ),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                // Pass config
                config
        );

        command = new RamseteCommand(
                turnTrajectory,
                diffDriveOdometry::getPoseMeters,
                new RamseteController(),
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
        gyro.reset();
        drivetrain.resetLeftPosition();
        drivetrain.resetRightPosition();

        command.schedule();
    }

    @Override
    public void execute() {
        diffDriveOdometry.update(gyro.getRotation2d(), drivetrain.getLeftPosition(), drivetrain.getRightPosition());
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.tankDriveVolts(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return command.isFinished();
    }
}
