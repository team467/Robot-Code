package frc.robot.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class HubTarget {
    private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("Vision").getSubTable("HubTarget");
    /**
     * @return If the camera sees the target
     */
    public static boolean hasTarget() {
        return table.getEntry("isValid").getBoolean(false);
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
     * @return Rotation2d to target
     */
    public static Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getAngle());
    }

    /**
     * @return Translation2d to target
     */
    public static Translation2d getTranslation2d() {
        return new Translation2d(getDistance(), getRotation2d());
    }
}
