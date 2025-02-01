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

  public GyroIOPigeon2() {
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(odometryFrequency);
    yawVelocity.setUpdateFrequency(50.0);
    pigeon.optimizeBusUtilization();
    yawTimestampQueue = OdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = OdometryThread.getInstance().registerSignal(yaw::getValueAsDouble);
    accelX.setUpdateFrequency(50.0);
    accelY.setUpdateFrequency(50.0);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(yaw, yawVelocity, accelX, accelY).equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
    inputs.previousVectorMagnitude = inputs.VectorMagnitude;
    // force the robot is moving at, calculated using the x and y acceleration values from pidgeon
    inputs.VectorMagnitude =
        Math.pow(
            Math.pow(accelX.getValueAsDouble() * 9.8, 2.0)
                + Math.pow(accelY.getValueAsDouble() * 9.8, 2.0),
            0.5);
    // difference in velocities from previous velocity and current velocity
    inputs.vectorDiff = (inputs.VectorMagnitude - inputs.previousVectorMagnitude);
    // records previous velocity before updating
    inputs.previousVectorAngle = inputs.VectorAngle;
    // using x and y accel values to see what direction the robot is heading
    inputs.VectorAngle =
        Math.atan(accelY.getValueAsDouble() * 9.8 / (accelX.getValueAsDouble() * 9.8));
    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
