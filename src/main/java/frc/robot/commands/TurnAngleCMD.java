package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gyro;
import frc.robot.utilities.MathUtils;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class TurnAngleCMD extends CommandBase {
private Drivetrain drivetrain;
private Gyro gyro;
private double angle;
DifferentialDriveKinematics diffDriveKinematics;
DifferentialDriveOdometry diffDriveOdometry;
DifferentialDriveVoltageConstraint autDriveVoltageConstraint;
TrajectoryConfig config;
Trajectory turnTrajectory;
RamseteCommand command;

public TurnAngleCMD(Drivetrain drivetrain, Gyro gyro, double angle) {
    this.drivetrain = drivetrain;
    this.gyro = gyro;
    this.angle = angle;

    diffDriveKinematics = new DifferentialDriveKinematics(0.58);
    diffDriveOdometry = new DifferentialDriveOdometry(gyro.getRotation2d());

    addRequirements(drivetrain);
    addRequirements(gyro);
}

@Override
public void initialize() {
    gyro.reset();
    diffDriveOdometry = new DifferentialDriveOdometry(gyro.getRotation2d());
    drivetrain.resetLeftPosition();
    drivetrain.resetRightPosition();

    autDriveVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(RobotConstants.get().driveForwardRightFF().getkS(),
                                       RobotConstants.get().driveForwardRightFF().getkV(),
                                       RobotConstants.get().driveForwardRightFF().getkA()),
            diffDriveKinematics,
            10);
    
    config =
        new TrajectoryConfig(RobotConstants.get().driveMaxVelocity(),
                RobotConstants.get().driveMaxAcceleration())
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(diffDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autDriveVoltageConstraint);
    

            turnTrajectory = TrajectoryGenerator.generateTrajectory(
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

    // TODO ONLY ONE CONSTANTS
    command = new RamseteCommand(
        turnTrajectory,
        diffDriveOdometry::getPoseMeters,
        new RamseteController(),
        new SimpleMotorFeedforward(RobotConstants.get().driveForwardRightFF().getkS(),
                                       RobotConstants.get().driveForwardRightFF().getkV(),
                                       RobotConstants.get().driveForwardRightFF().getkA()),
        diffDriveKinematics,
        drivetrain::getWheelSpeeds,
        new PIDController(RobotConstants.get().driveForwardLeftPositionFB().getkP(), 0, 0),
        new PIDController(RobotConstants.get().driveForwardLeftPositionFB().getkP(), 0, 0),
        // RamseteCommand passes volts to the callback
        drivetrain::tankDriveVolts
    );

    command.schedule();
}

@Override
public void execute() {
    diffDriveOdometry.update(gyro.getRotation2d(), drivetrain.getLeftPosition(), drivetrain.getRightPosition());
}

@Override
public void end(boolean interrupted) {
    drivetrain.arcadeDrive(0, 0);
}

// Returns true when the command should end.
@Override
public boolean isFinished() {
    return command.isFinished();
}
}
