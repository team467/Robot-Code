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
import frc.robot.RobotConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gyro;
import frc.robot.vision.HubTarget;

import java.util.List;

public class TurnToTargetCMD extends CommandBase {
    private final Drivetrain drivetrain;
    private final Gyro gyro;
    private Command command;

    public TurnToTargetCMD(Drivetrain drivetrain, Gyro gyro) {
        this.drivetrain = drivetrain;
        this.gyro = gyro;
    }

    @Override
    public void initialize() {
        if (HubTarget.hasTarget() && Math.abs(HubTarget.getAngle()) < Units.radiansToDegrees(Math.atan2(0.6, HubTarget.getDistance()))) {
            command = new GoToDistanceAngleCMD(drivetrain, gyro, -RobotConstants.get().driveKinematics().trackWidthMeters/2, HubTarget.getAngle(), true);
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
