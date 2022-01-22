package frc.robot.tuning;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Tuner {
    public String getTunerName();
    public Subsystem[] getTunerSubsystems();
    public TunerParameter[] getTunerParameters();
    public void addTunerParameter(String parameterName, TunerParameter tunerParameter);
    public TunerParameter getTunerParameter(String parameterName);
    public void addTunerButton(String buttonName, TunerButton tunerParameter);
    public TunerButton getTunerButton(String buttonName);
    public void initalizeTunerNetworkTables();
    public void initalizeTuner();
}
