package frc.robot.loggers;

import com.revrobotics.spark.SparkMax;
import org.wpilib.epilogue.CustomLoggerFor;
import org.wpilib.epilogue.logging.ClassSpecificLogger;
import org.wpilib.telemetry.TelemetryTable;

@CustomLoggerFor(SparkMax.class)
public class SparkLogger extends ClassSpecificLogger<SparkMax> {

  public SparkLogger() {
    super(SparkMax.class);
  }

  @Override
  public void update(TelemetryTable table, SparkMax object) {
    table.log("Temperature (C)", object.getMotorTemperature().get());
    table.log("Applied Volts", object.getAppliedOutput().get() * object.getBusVoltage().get());
    table.log("Amps", object.getOutputCurrent().get());

    var encoder = object.getEncoder();
    table.log("Encoder Position (Rotations)", encoder.getPosition().get());
    table.log("Encoder Velocity (RPM)", encoder.getVelocity().get());
  }
}
