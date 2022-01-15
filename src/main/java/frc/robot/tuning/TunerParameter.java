package frc.robot.tuning;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableType;

public interface TunerParameter {
    public String getName();
    public NetworkTableEntry getEntry();
    public NetworkTableType getType();
    
    public void setDefaultValue(ParameterValue value);
    public void setValue(ParameterValue value);
    public ParameterValue getValue();
}
