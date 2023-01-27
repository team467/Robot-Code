package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotConstants;
import frc.robot.subsystems.arm.Arm;

public class Arm2023ManualUpCMD extends CommandBase {
    private final arm Arm;
    
    public Arm2023ManualUpCMD(Arm arm) {
        this.Arm = arm;

        addRequirements(arm);
    }


    @Override 
    public void execute() {
        Arm.up();
    }

    @Override 
    public void end(boolean interrupted) {}


    @Override
    public boolean isFinished() {
        return false;
    }
    
}
