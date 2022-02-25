package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotConstants;

public class HubTarget {
    private static final Translation2d hubTranslation = new Translation2d(Units.inchesToMeters(324), Units.inchesToMeters(162));
    private static final double hubOffset = Units.inchesToMeters(26.6875);

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

    /**
     * @return Translation2d to center of target
     */
    public static Translation2d getCenterTranslation2d() {
        return new Translation2d(getDistance() + hubOffset, getRotation2d());
    }

    /**
     * @return Translation2d of robot absolute to field
     */
    public static Translation2d getRobotTranslation() {
        return hubTranslation.minus(getCenterTranslation2d()).minus(RobotConstants.get().hubCameraOffset());
    }

    /**
     * @return Timestamp of frame
     */
    public static double getTimestamp() {
        // TODO get frame timestamp / time offset
        double offset = 0;
        return Timer.getFPGATimestamp() + offset;
    }
}
