package frc.lib.constantcontrol;

/** Fancy constant wrapper for a gear ratio */
public class GearRatio {
  private final double output;
  private final double input;

  /**
   * Configure a gear ratio
   *
   * @param output gear output
   * @param input gear input
   */
  public GearRatio(double output, double input) {
    this.output = output;
    this.input = input;
  }

  /** Configure a 1:1 gear ration */
  public GearRatio() {
    this(1, 1);
  }

  /**
   * Get output
   *
   * @return output
   */
  public double getOutput() {
    return output;
  }

  /**
   * Get input
   *
   * @return input
   */
  public double getInput() {
    return input;
  }

  /**
   * Finds the rotations per output
   *
   * @return rotations per output
   */
  public double getRotationsPerOutput() {
    return output / input;
  }

  /**
   * Finds the rotations per input
   *
   * @return rotations per input
   */
  public double getRotationsPerInput() {
    return input / output;
  }
}
