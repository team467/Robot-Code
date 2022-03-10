package frc.robot.utilities;

/**
 * Collection of useful math functions.
 */
public class MathUtils {

  private MathUtils() {
    throw new IllegalStateException("Utility class");
  }

  /**
   * Finds the weighted average of a and b
   * @param a a value
   * @param b a value
   * @param weight the weight to apply
   * @return the weighted average of a and b
   */
  public static double weightedAverage(double a, double b, double weight) {
    return a * (1 - weight) + b * weight;
  }
}
