package frc.robot.commands.arm;

import frc.robot.subsystems.arm.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RotateDown extends CommandBase {
    private final Arm arm;

    public RotateDown(Arm arm) {
        this.arm = arm;

        addRequirements(this.arm);
    }

    @Override
    public void initialize() {
    }


    @Override 
    public void execute() {
        arm.rotateTargetDegrees;
    }


    @Override
    public void end(boolean interrupted) {}


    @Override 
    public boolean isFinished() {
        return arm.isStopped();
    }
}