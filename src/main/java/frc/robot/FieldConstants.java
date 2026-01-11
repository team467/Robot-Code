package frc.robot;

import edu.wpi.first.math.geometry.*;

/**
 * Contains various field dimensions and useful reference points. Dimensions are in meters, and sets
 * of corners start in the lower left moving clockwise.
 *
 * <p>All translations and poses are stored with the origin at the rightmost point on the BLUE
 * ALLIANCE wall. Use the {@link frc.lib.utils.AllianceFlipUtil#apply(Translation2d)} and {@link
 * frc.lib.utils.AllianceFlipUtil#apply(Pose2d)} methods to flip these values based on the current
 * alliance color.
 */
public class FieldConstants {

  public static double fieldLength;
  public static double fieldWidth;
}
