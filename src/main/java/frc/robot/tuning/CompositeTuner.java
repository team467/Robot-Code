package frc.robot.tuning;

import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Subsystem;

public abstract class CompositeTuner implements Tuner {
    HashMap<String, TunerParameter> tunerParameters = new HashMap<>();
    HashMap<String, TunerButton> tunerButtons = new HashMap<>();
    ArrayList<Subsystem> subsystems = new ArrayList<>();

    public CompositeTuner() {
        super();

        TunerManager.getTunerManager().registerTuner(this);
    }

    abstract public String getName();

    @Override
    public String getTunerName() {
        return this.getName() + " Tuner";
    }

    @Override
    public Subsystem[] getTunerSubsystems() {
        Subsystem[] array = new Subsystem[subsystems.size()];
        return subsystems.toArray(array);
    }

    public void addSubsystem(Subsystem subsystem) {
        subsystems.add(subsystem);
    }
    
    @Override
    public TunerParameter[] getTunerParameters() {
        TunerParameter[] tunerArray = new TunerParameter[tunerParameters.size()];
        tunerArray = tunerParameters.values().toArray(tunerArray);
        return tunerArray;
    }

    @Override
    public void addTunerParameter(String parameterName, TunerParameter tunerParameter) {
        tunerParameters.put(parameterName, tunerParameter);
    }
    

    @Override
    public TunerParameter getTunerParameter(String parameterName) {
        return tunerParameters.get(parameterName);
    }

    @Override
    public void addTunerButton(String buttonName, TunerButton tunerButton) {
        tunerButtons.put(buttonName, tunerButton);
    }
    

    @Override
    public TunerButton getTunerButton(String buttonName) {
        return tunerButtons.get(buttonName);
    }
}
