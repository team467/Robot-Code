package frc.robot.tuning;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ArcadeDriveCMD;

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
