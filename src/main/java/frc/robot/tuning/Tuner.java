package frc.robot.tuning;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Tuner {
    public String getTunerName();
    public Subsystem[] getTunerSubsystems();
    public TunerParameter[] getTunerParameters();
    public void addTunerParameter(String tunerName, TunerParameter tunerParameter);
    public TunerParameter getTunerParameter(String tunerName);
    public void initalizeTunerNetworkTables();
    public void initalizeTuner();
}
