package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gyro;

public class TurnToAngleCMD extends CommandBase {
    GoToTrajectoryCMD command;

    public TurnToAngleCMD(Drivetrain drivetrain, Gyro gyro, double angle) {
        command = new GoToTrajectoryCMD(drivetrain, gyro, new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(0, 0)), new Pose2d(0, 0, new Rotation2d(angle)));
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
