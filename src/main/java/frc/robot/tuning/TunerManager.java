package frc.robot.tuning;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TunerManager {
    private static TunerManager instance = null;
    private ArrayList<Tuner> tuners = new ArrayList<>();
    private SendableChooser<Tuner> tunerChooser = new SendableChooser<>();

    private TunerManager() {}

    public void registerTuner(Tuner tuner) {
        // System.out.println("Reigstered tuner: " + tuner.getTunerName());
        tuners.add(tuner);
        tuner.initalizeTunerNetworkTables();
        tunerChooser.addOption(tuner.getTunerName(), tuner);
    }

    public void addChooserToNT() {
        SmartDashboard.putData(tunerChooser);
    }

    public Tuner getTunerChoice() {
        return tunerChooser.getSelected();
    }

    public static TunerManager getTunerManager() {
        if (instance == null) {
            instance = new TunerManager();
        }

        return instance;
    }
}
