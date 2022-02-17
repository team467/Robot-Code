package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gyro;

public class GoToDistanceAngleCMD extends CommandBase {
    GoToTrajectoryCMD command;

    public GoToDistanceAngleCMD(Drivetrain drivetrain, Gyro gyro, double distance, double angle) {
        // Angle should be negative because ccw positve
        // Translation angle is most likely off of horiztonal 
        // TODO, fix to it off vertical
        Translation2d output = new Translation2d(distance, Rotation2d.fromDegrees(angle));
        System.out.println(String.format("x: %f, y:%f", output.getX(), output.getY()));
        command = new GoToTrajectoryCMD(drivetrain, gyro, new Pose2d(0, 0, new Rotation2d()), List.of(), new Pose2d(output.getY(), -output.getX(), Rotation2d.fromDegrees(angle)));
    }

    @Override
    public void initialize() {
        command.schedule();
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }
    
}
