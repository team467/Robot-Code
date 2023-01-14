package frc.lib.utils;

/** Collection of useful math functions. */
public class MathUtils {

  private MathUtils() {
    throw new IllegalStateException("Utility class");
  }

  /**
   * Returns the weighted average of two numbers.
   *
   * @param a The first number to average
   * @param b the current value of the variable you're trying to change
   * @param weight The weight of the second value.
   * @return The weighted average of a and b.
   */
  public static double weightedAverage(double a, double b, double weight) {
    return a * (1 - weight) + b * weight;
  }
}
