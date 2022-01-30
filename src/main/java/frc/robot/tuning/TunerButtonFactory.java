package frc.robot.tuning;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class TunerButtonFactory {
    public static TunerButton create(String name, Tuner tuner) {
        TunerButton tunerButton = new TunerButton() {
            private String buttonName = name;
            private NetworkTableEntry tableEntry = NetworkTableInstance.getDefault().getTable("tuning").getSubTable(tuner.getTunerName()).getEntry(getName());
            
            @Override
            public String getName() {
                return buttonName + " Button";
            }

            @Override
            public NetworkTableEntry getEntry() {
                return tableEntry;
            }
        };

        tunerButton.unpress();
        return tunerButton;
    }
}
