package frc.robot.loggers;

import com.ctre.phoenix6.hardware.TalonFX;
import org.wpilib.epilogue.CustomLoggerFor;
import org.wpilib.epilogue.logging.ClassSpecificLogger;

@CustomLoggerFor(TalonFX.class)
public class TalonLogger extends ClassSpecificLogger<TalonFX> {

  public TalonLogger() {
    super(TalonFX.class);
  }

  @Override
  public void update(org.wpilib.telemetry.TelemetryTable table, TalonFX object) {
    table.log("Velocity (RPM)", object.getVelocity().getValue().times(60));
    table.log("Amps", object.getStatorCurrent().getValue());
    table.log("Volts", object.getMotorVoltage().getValue());
  }
}
