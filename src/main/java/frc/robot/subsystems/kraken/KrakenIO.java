package frc.robot.subsystems.kraken;

public interface KrakenIO {
  public class KrakenIOInputs {
    public double appliedVoltage;
  }
  public default void updateInputs(KrakenIOInputs inputs) {}
  public default void setVoltage(double volts) {}

}
