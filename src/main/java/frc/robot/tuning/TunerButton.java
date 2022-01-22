package frc.robot.tuning;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.button.Button;

public abstract class TunerButton extends Button {
    public abstract String getName();
    public abstract NetworkTableEntry getEntry();
    
    @Override
    public boolean get() {
        return getEntry().getBoolean(false);
    }
}
