package frc.robot.tuning;

import edu.wpi.first.networktables.NetworkTableType;

public abstract class ParameterValue {
    public NetworkTableType getType() { return NetworkTableType.kUnassigned; }
    public boolean getBoolean() { return false; }
    public double getDouble() { return 0d; }
    public String getString() { return ""; }
    public byte[] getRaw() { return new byte[0]; }
    public boolean[] getBooleanArray() { return new boolean[0]; }
    public double[] getDoubleArray() { return new double[0]; }
    public String[] getStringArray() { return new String[0]; }
}
