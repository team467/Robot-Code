package frc.robot.tuning;

import java.util.HashMap;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class SubsystemTuner extends SubsystemBase implements Tuner {
    private HashMap<String, NetworkTableEntry> entries = new HashMap<>();

    public SubsystemTuner() {
        super();

        TunerManager.getTunerManager().registerTuner(this);
    }

    @Override
    public String getTunerName() {
        return this.getName() + " Tuner";
    }

    @Override
    public Subsystem[] getTunerSubsystems() {
        return new Subsystem[]{this};
    }

    @Override
    public void addEntry(String name, NetworkTableEntry entry) {
        entries.put(name, entry);
    }

    @Override
    public NetworkTableEntry getEntry(String name) {
        return entries.get(name);
    }

    @Override
    public void initializeTuner() {
        CommandScheduler.getInstance().clearButtons();
    }
}
