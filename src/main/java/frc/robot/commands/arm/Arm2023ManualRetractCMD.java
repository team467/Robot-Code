package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotConstants;
import frc.robot.subsystems.arm.Arm;

public class Arm2023ManualRetractCMD extends CommandBase {
    private Arm arm;

    public Arm2023ManualRetractCMD(Arm arm) {
        this.arm = arm;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // climber.retract();
        // retract does not exist, have to find what it is called in codes
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
