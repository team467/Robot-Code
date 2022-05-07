package frc.robot.tuning;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Tuner {
    String getTunerName();
    Subsystem[] getTunerSubsystems();
    void initializeTunerNetworkTables(ShuffleboardTab tab);
    void initializeTuner();
    void addEntry(String name, NetworkTableEntry entry);
    NetworkTableEntry getEntry(String name);
}
