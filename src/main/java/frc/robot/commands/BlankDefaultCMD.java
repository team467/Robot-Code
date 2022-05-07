package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class BlankDefaultCMD extends CommandBase {
    public BlankDefaultCMD(Subsystem subsystem) {
        addRequirements(subsystem);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
