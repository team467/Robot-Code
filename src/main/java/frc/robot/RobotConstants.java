package frc.robot;

import frc.robot.constants.BlankConstants;
import frc.robot.constants.Constants;

public class RobotConstants {
    private static Constants constants = new BlankConstants();
    
    public static Constants get() {
        return constants;
    }

    public static void set(Constants robot) {
        constants = robot;
    }
}
