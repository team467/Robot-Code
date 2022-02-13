package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gyro;
import frc.robot.utilities.MathUtils;

public class TurnAngleCMD extends CommandBase {
private Drivetrain drivetrain;
private Gyro gyro;
private double angle;
DifferentialDriveKinematics diffDriveKinematics;
DifferentialDriveOdometry diffDriveOdometry;
DifferentialDriveVoltageConstraint autDriveVoltageConstraint;
TrajectoryConfig config;

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
    
            
}

@Override
public void execute() {
    diffDriveOdometry.update(gyro.getRotation2d(), drivetrain.getLeftPosition(), drivetrain.getRightPosition());
    drivetrain.arcadeDrive(0, direction * MathUtil.clamp(Math.abs(error)/10, 0.1, 0.5));
}

@Override
public void end(boolean interrupted) {
    drivetrain.arcadeDrive(0, 0);
}

// Returns true when the command should end.
@Override
public boolean isFinished() {
    return Math.abs(angle - gyro.getNextAngle()) < 0.1;
}
}
