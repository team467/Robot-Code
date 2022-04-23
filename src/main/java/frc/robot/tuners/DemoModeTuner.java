package frc.robot.tuners;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.RobotContainer;
import frc.robot.tuning.CompositeTuner;

public class DemoModeTuner extends CompositeTuner {
    private final RobotContainer robotContainer;

    public DemoModeTuner(RobotContainer robotContainer) {
        super();
        this.robotContainer = robotContainer;
    }

    @Override
    public void initializeTunerNetworkTables(ShuffleboardTab tab) {

    }

    @Override
    public void initializeTuner() {
        robotContainer.enableDemoMode();
        robotContainer.configureButtonBindings();
    }

    @Override
    public String getName() {
        return "Demo Mode";
    }
    
}
