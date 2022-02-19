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
    boolean finished = false;

    public GoToDistanceAngleCMD(Drivetrain drivetrain, Gyro gyro, double distance, double angle) {
        Translation2d output = new Translation2d(distance, Rotation2d.fromDegrees(angle));
        if (distance > 0) {
            command = new GoToTrajectoryCMD(drivetrain, gyro, new Pose2d(0, 0, new Rotation2d()), List.of(), new Pose2d(output.getX(), -output.getY(), Rotation2d.fromDegrees(-angle)));
        } else {
            finished = true;
        }
    }

    @Override
    public void initialize() {
        if (!finished) {
            command.schedule();
        }
    }

    @Override
    public boolean isFinished() {
        return command.isFinished() || finished;
    }

}
