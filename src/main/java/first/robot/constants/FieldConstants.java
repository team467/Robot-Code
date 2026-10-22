package first.robot.constants;

import java.util.Optional;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.shape.Rectangle2d;
import org.wpilib.math.util.Units;

public final class FieldConstants {
  public static final double fieldLength = 16.541;
  public static final double fieldWidth = 8.069;

  public static final class Hub {
    public static final Translation2d blueCenter = new Translation2d(4.6255178, 4.0346376);

    public static final Translation3d innerCenterPoint =
        new Translation3d(blueCenter.getX(), blueCenter.getY(), Units.inchesToMeters(72.0));

    public static final Pose2d nearFace = new Pose2d(4.0218614, 4.0346376, Rotation2d.PI);

    private Hub() {}
  }

  public static final Pose2d BLUE_HUB_CENTER = new Pose2d(Hub.blueCenter, Rotation2d.ZERO);
  public static final Pose2d RED_HUB_CENTER =
      new Pose2d(fieldLength - Hub.blueCenter.getX(), fieldWidth - Hub.blueCenter.getY(), Rotation2d.PI);

  public static final Rectangle2d NEUTRAL_ZONE =
      new Rectangle2d(
          new Translation2d(BLUE_HUB_CENTER.getX(), 0.0),
          new Translation2d(RED_HUB_CENTER.getX(), fieldWidth));

  public static final Rectangle2d BLUE_ZONE =
      new Rectangle2d(new Translation2d(0.0, 0.0), new Translation2d(BLUE_HUB_CENTER.getX(), fieldWidth));
  public static final Rectangle2d RED_ZONE =
      new Rectangle2d(
          new Translation2d(RED_HUB_CENTER.getX(), 0.0), new Translation2d(fieldLength, fieldWidth));

  public static Optional<Pose2d> targetHub() {
    return MatchState.getAlliance()
        .map(
            a -> {
              return switch (a) {
                case RED -> RED_HUB_CENTER;
                case BLUE -> BLUE_HUB_CENTER;
              };
            });
  }

  private FieldConstants() {}
}
