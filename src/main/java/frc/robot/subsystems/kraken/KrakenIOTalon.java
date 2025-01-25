package frc.robot.subsystems.kraken;

import com.ctre.phoenix6.hardware.TalonFX;

public class KrakenIOTalon implements KrakenIO {
  private final TalonFX talon;

  public KrakenIOTalon(int motorID) {
    talon = new TalonFX(motorID);
  }

  @Override
  public void updateInputs(KrakenIOInputs inputs) {
    inputs.appliedVoltage = talon.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    talon.setVoltage(volts);
  }
}
