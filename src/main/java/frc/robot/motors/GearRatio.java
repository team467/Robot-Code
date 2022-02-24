package frc.robot.motors;

public class GearRatio {
  private final double driven;
  private final double driving;

  public GearRatio(double driven, double driving) {
    this.driven = driven;
    this.driving = driving;
  }

  public GearRatio() {
    this(1, 1);
  }

  public double getDriven() {
    return driven;
  }

  public double getDriving() {
    return driving;
  }

  public double getRotationsPerDriven() {
    return driven/driving;
  }

  public double getRotationsPerDriving() {
    return driving/driven;
  }
}
