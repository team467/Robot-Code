package frc.robot.tuning;

import java.util.ArrayList;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TunerManager {
    private static TunerManager instance = null;
    private ArrayList<Tuner> tuners = new ArrayList<>();
    private SendableChooser<Tuner> tunerChooser = new SendableChooser<>();

    private TunerManager() {
    }

    public void registerTuner(Tuner tuner) {
        // System.out.println("Reigstered tuner: " + tuner.getTunerName());
        Shuffleboard.getTab(tuner.getTunerName()).add("Tuner Selection", tunerChooser).withWidget(BuiltInWidgets.kComboBoxChooser)
                .withSize(2, 1)
                .withPosition(0, 0);
        tuners.add(tuner);
        tuner.initalizeTunerNetworkTables(Shuffleboard.getTab(tuner.getTunerName()));
        tunerChooser.addOption(tuner.getTunerName(), tuner);
    }

    public void addChooserToNT() {
        
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
