package frc.lib.mechanisms;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.units.measure.Angle;

public class PositioningSpark implements PositionControlled {

  private final SparkMax spark;

  public PositioningSpark(CANPort canPort, int port, boolean invert) {
    spark = new SparkMax(canPort, port, MotorType.kBrushless);
  }

  @Override
  public void goToPosition(Angle position) {

  }

  @Override
  public boolean atSetpoint() {
    return false;
  }

  @Override
  public void stop() {

  }
}
