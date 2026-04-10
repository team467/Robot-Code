package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.utils.AllianceFlipUtil;
import java.util.Objects;
import java.util.function.Supplier;

public class Zone {
  public record Tuple2d(double x, double y) {}

  private Tuple2d minCorner;
  private Tuple2d maxCorner;
  private final Supplier<Pose2d> poseSupplier;

  public Zone(Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = Objects.requireNonNull(poseSupplier, "poseSupplier");
  }

  public Zone(Supplier<Pose2d> poseSupplier, Tuple2d cornerA, Tuple2d cornerB) {
    this(poseSupplier);
    initializeZone(cornerA, cornerB);
  }

  public static Tuple2d corner(double x, double y) {
    return new Tuple2d(x, y);
  }

  public void initializeZone(Tuple2d cornerA, Tuple2d cornerB) {
    Objects.requireNonNull(cornerA, "cornerA");
    Objects.requireNonNull(cornerB, "cornerB");

    double minX = Math.min(cornerA.x(), cornerB.x());
    double maxX = Math.max(cornerA.x(), cornerB.x());
    double minY = Math.min(cornerA.y(), cornerB.y());
    double maxY = Math.max(cornerA.y(), cornerB.y());

    this.minCorner = new Tuple2d(minX, minY);
    this.maxCorner = new Tuple2d(maxX, maxY);
  }

  public boolean isInitialized() {
    return minCorner != null && maxCorner != null;
  }

  public boolean isInZone() {
    return contains(poseSupplier.get());
  }

  public boolean isInZoneForAlliance() {
    return containsForAlliance(poseSupplier.get());
  }

  public boolean contains(Pose2d pose) {
    Objects.requireNonNull(pose, "pose");
    return contains(pose.getX(), pose.getY());
  }

  public boolean containsForAlliance(Pose2d pose) {
    Objects.requireNonNull(pose, "pose");
    return containsForAlliance(pose.getX(), pose.getY());
  }

  public boolean contains(double x, double y) {
    if (!isInitialized()) {
      return false;
    }

    return x >= minCorner.x() && x <= maxCorner.x() && y >= minCorner.y() && y <= maxCorner.y();
  }

  public boolean containsForAlliance(double x, double y) {
    if (!isInitialized()) {
      return false;
    }

    double minX = minCorner.x();
    double maxX = maxCorner.x();
    double minY = minCorner.y();
    double maxY = maxCorner.y();

    if (AllianceFlipUtil.shouldFlip()) {
      double flippedMinX = AllianceFlipUtil.applyX(minX);
      double flippedMaxX = AllianceFlipUtil.applyX(maxX);
      double flippedMinY = AllianceFlipUtil.applyY(minY);
      double flippedMaxY = AllianceFlipUtil.applyY(maxY);

      minX = Math.min(flippedMinX, flippedMaxX);
      maxX = Math.max(flippedMinX, flippedMaxX);
      minY = Math.min(flippedMinY, flippedMaxY);
      maxY = Math.max(flippedMinY, flippedMaxY);
    }

    return x >= minX && x <= maxX && y >= minY && y <= maxY;
  }
}
