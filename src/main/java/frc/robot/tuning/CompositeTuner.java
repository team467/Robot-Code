package frc.robot.tuning;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface CompositeTuner {
    public String getName();
    public Subsystem[] getSubsystems();
    public TunerParameter[] getTunerParameters();
    public void update();
}
