package frc.robot.motors;

public class GearRatio {
  private final double driving;
  private final double driven;

  public GearRatio(double driving, double driven) {
    this.driving = driving;
    this.driven = driven;
  }

  public GearRatio() {
    this(1, 1);
  }

  public double getDriving() {
    return driving;
  }

  public double getDriven() {
    return driven;
  }

  public double getRotationsPerDriving() {
    return driving/driven;
  }

  public double getRotationsPerDriven() {
    return driven/driving;
  }
}
