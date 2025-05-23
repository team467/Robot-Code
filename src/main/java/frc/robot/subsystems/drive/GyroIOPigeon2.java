package frc.robot.subsystems.drive;

import static frc.robot.Schematic.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import java.util.Queue;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
  private final Alert impactAlert =
      new Alert("Impact Detected, lowering elevator to prevent flipping.", AlertType.kWarning);
  private final Pigeon2 pigeon = new Pigeon2(pigeonCanId);
  private final StatusSignal<Angle> yaw = pigeon.getYaw();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();
  private final StatusSignal<LinearAcceleration> accelX = pigeon.getAccelerationX();
  private final StatusSignal<LinearAcceleration> accelY = pigeon.getAccelerationY();
  private final StatusSignal<Angle> robotPitch = pigeon.getPitch();
  private final StatusSignal<Angle> robotRoll = pigeon.getRoll();

  public GyroIOPigeon2() {
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(odometryFrequency);
    yawVelocity.setUpdateFrequency(50.0);
    pigeon.optimizeBusUtilization();
    yawTimestampQueue = OdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = OdometryThread.getInstance().registerSignal(yaw::getValueAsDouble);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, accelY, accelX, robotPitch, robotRoll);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(yaw, yawVelocity, accelX, accelY, robotRoll, robotPitch)
            .equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    inputs.roll = robotRoll.getValueAsDouble();
    inputs.pitch = robotPitch.getValueAsDouble();

    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
