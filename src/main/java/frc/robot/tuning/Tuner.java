package frc.robot.tuning;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Tuner {
    public String getTunerName();
    public Subsystem[] getTunerSubsystems();
    public void initalizeTunerNetworkTables(ShuffleboardTab tab);
    public void initalizeTuner();
    public void addEntry(String name, NetworkTableEntry entry);
    public NetworkTableEntry getEntry(String name);
}
