package frc.robot.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class HubTarget {
    private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("Vision").getSubTable("HubTarget");

    /**
     * Returns calculated flywheel speed in rad/s from any distance in meters
     * @param distance Distance in meters
     * @return Calculated flywheel speed in rad/s
     */
    public static double getFlywheelVelocity(double distance) {
        return 108.5792974 + (141.8090384 * distance);
    }

    /**
     * Returns calculated flywheel speed in rad/s from the distance to the target
     * @return Calculated flywheel speed in rad/s
     */
    public static double getFlywheelVelocity() {
        return getFlywheelVelocity(getDistance());
    }


    /**
     * @return Distance to the target in meters
     */
    public static double getDistance() {
        return table.getEntry("distance").getDouble(0) * 0.3048;
    }


    /**
     * @return Angle to target off vertical in degrees
     */
    public static double getAngle() {
        return table.getEntry("angle").getDouble(0);
    }


    /**
     * @return If the camera sees the target
     */
    public static boolean hasTarget() {
        return table.getEntry("isValid").getBoolean(false);
    }
}
