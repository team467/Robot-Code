package frc.robot.subsystems.stereoVision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;

public class stereoVisionConstants {

  public static double focalLength = 457; // pixels
  public static double baseLength = Units.inchesToMeters(6); // meters
  public static double cameraHeight = Units.inchesToMeters(10); // meters

  public static Transform2d toRobotCenter = new Transform2d(0, 0, new Rotation2d());
}
