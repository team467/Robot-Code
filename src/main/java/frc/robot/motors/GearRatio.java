package frc.robot.motors;

public class GearRatio {
  private final double output;
  private final double input;

  public GearRatio(double output, double input) {
    this.output = output;
    this.input = input;
  }

  public GearRatio() {
    this(1, 1);
  }

  public double getOutput() {
    return output;
  }

  public double getInput() {
    return input;
  }

  public double getRotationsPerOutput() {
    return output / input;
  }

  public double getRotationsPerInput() {
    return input / output;
  }
}
