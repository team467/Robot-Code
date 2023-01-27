package frc.robot.commands.arm;

import frc.robot.subsystems.arm.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExtendLow extends CommandBase {
    private final Arm arm;

    public ExtendLow(Arm arm) {
        this.arm = arm;

        addRequirements(this.arm);
    }

    @Override
    public void initialize() {
    }


    @Override 
    public void execute() {
        arm.extendAndRotate(0, 0);
       
    }

    @Override
    public void end(boolean interrupted) {}


    @Override 
    public boolean isFinished() {
        return arm.isStopped();
    }
}