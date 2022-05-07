package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gyro;
import frc.robot.vision.HubTarget;

import java.util.List;

public class GoToTargetCMD extends CommandBase {
    private final Drivetrain drivetrain;
    private final Gyro gyro;
    private Command command;

    public GoToTargetCMD(Drivetrain drivetrain, Gyro gyro) {
        this.drivetrain = drivetrain;
        this.gyro = gyro;
    }

    @Override
    public void initialize() {
        if (HubTarget.hasTarget()) {
            Translation2d output = HubTarget.getTranslation2d().minus(new Translation2d(1, HubTarget.getRotation2d().unaryMinus()));
            command = new GoToTrajectoryCMD(drivetrain, gyro, new Pose2d(0, 0, new Rotation2d()), List.of(), new Pose2d(output.getX(), -output.getY(), HubTarget.getRotation2d().unaryMinus()), false);
            command.schedule();
        } else {
            command = new InstantCommand();
        }
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }
}
