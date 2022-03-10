package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.motors.MotorControllerEncoder;
import frc.robot.motors.MotorControllerFactory;
import frc.robot.motors.MotorType;

public class Climber2020 extends SubsystemBase {
  private final MotorControllerEncoder climberMotor;
  private boolean enabled = false;

  public Climber2020() {
    super();

    climberMotor =
        MotorControllerFactory.create(
            RobotConstants.get().climber2020MotorId(), MotorType.SPARK_MAX_BRUSHLESS);

    climberMotor.setInverted(RobotConstants.get().climber2020MotorInverted());
  }

  /** Enables the climber */
  public void enable() {
    enabled = true;
  }

  /** Disables the climber */
  public void disable() {
    enabled = false;
  }

  /**
   * Checks if the climber is enabled
   *
   * @return if the climber is enabled
   */
  public boolean isEnabled() {
    return enabled;
  }

  /** Moves the climber arms up */
  public void up() {
    if (this.isEnabled()) {
      climberMotor.set(RobotConstants.get().climber2022UpSpeed());
    }
  }

  /** Moves the climber arms downs */
  public void down() {
    if (this.isEnabled()) {
      climberMotor.set(RobotConstants.get().climber2022DownSpeed());
    }
  }

  /** Stops the climber arms */
  public void stop() {
    climberMotor.set(0);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("Climber Position", () -> climberMotor.getPosition(), null);
    builder.addDoubleProperty("Climber Velocity", () -> climberMotor.getVelocity(), null);
  }
}
