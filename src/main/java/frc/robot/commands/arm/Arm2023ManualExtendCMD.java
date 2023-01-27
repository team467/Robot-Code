package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotConstants;
import frc.robot.subsystems.arm.Arm;

public class Arm2023ManualExtendCMD extends CommandBase{
    private Arm arm;

    public Arm2023ManualExtendCMD(Arm arm) {
        this.arm = arm;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        climber.extend();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
