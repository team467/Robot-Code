package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DrivetrainNoneCMD extends CommandBase {
    public DrivetrainNoneCMD(Drivetrain drivetrain) {
        addRequirements(drivetrain);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
