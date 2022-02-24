package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gyro;
import frc.robot.vision.BallTracking;

public class GoToBallCMD extends CommandBase {
    // TODO: Create ball detection class
    private final Drivetrain drivetrain;
    private final Gyro gyro;
    private Command command;

    public GoToBallCMD(Drivetrain drivetrain, Gyro gyro) {
        this.drivetrain = drivetrain;
        this.gyro = gyro;
    }

    @Override
    public void initialize() {
        command = new GoToDistanceAngleCMD(drivetrain, gyro,
            BallTracking.getDistance(), BallTracking.getAngle(), false);
        command.schedule();
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }

}
