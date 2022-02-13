package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.logging.RobotLogManager;
import frc.robot.motors.MotorControllerEncoder;
import frc.robot.motors.MotorControllerFactory;
import frc.robot.motors.MotorType;
import org.apache.logging.log4j.Logger;

/** The llama neck subsystem, contains the llama neck motors and the limit switches. */
public class LlamaNeck2022 extends SubsystemBase {
  private static final Logger LOGGER = RobotLogManager.getMainLogger(LlamaNeck2022.class.getName());

  private final MotorControllerEncoder llamaNeckMotor;
  private final DigitalInput upperLimitSwitch;
  private final DigitalInput lowerLimitSwitch;

  /** The llama neck subsystem, contains the llama neck motors and the limit switches. */
  public LlamaNeck2022() {
    super();

    llamaNeckMotor =
        MotorControllerFactory.create(
            RobotConstants.get().llamaNeck2022MotorID(), MotorType.SPARK_MAX_BRUSHLESS);
    llamaNeckMotor.setInverted(RobotConstants.get().llamaNeck2022MotorInverted());
    // upper switch, near the indexer wheel
    upperLimitSwitch =
        new DigitalInput(RobotConstants.get().llamaNeck2022UpperLimitSwitchChannel());
    // lower switch, gives upper ball enough breathing room
    lowerLimitSwitch =
        new DigitalInput(RobotConstants.get().llamaNeck2022LowerLimitSwitchChannel());
  }

  /**
   * Gets the status of the upper limit switch.
   *
   * @return true if pressed, false if not pressed
   */
  public boolean upperLimitSwitchIsPressed() {
    return !upperLimitSwitch.get();
  }

  /**
   * Gets the status of the lower limit switch.
   *
   * @return true if pressed, false if not pressed
   */
  public boolean lowerLimitSwitchIsPressed() {
    return !lowerLimitSwitch.get();
  }

  /** Idles the llama neck wheels. */
  public void idle() {
    LOGGER.debug(
        "Starting llamaNeck, setting speed to " + RobotConstants.get().llamaNeck2022IdleSpeed());
    llamaNeckMotor.set(RobotConstants.get().llamaNeck2022IdleSpeed());
  }

  /** Moves the llama neck wheels at a faster rate. */
  public void forward() {
    LOGGER.debug(
        "Starting llamaNeck, setting speed to " + RobotConstants.get().llamaNeck2022InSpeed());
    llamaNeckMotor.set(RobotConstants.get().llamaNeck2022InSpeed());
  }

  /** Moves the llama neck wheels backwards. */
  public void backward() {
    LOGGER.debug(
        "Reversing index, setting speed to " + RobotConstants.get().llamaNeck2022OutSpeed());
    llamaNeckMotor.set(-RobotConstants.get().llamaNeck2022OutSpeed());
  }

  /** Stops the llama neck wheels. */
  public void stop() {
    LOGGER.debug("Stopping llamaNeck, setting speed to 0");
    llamaNeckMotor.set(0.0);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addBooleanProperty("Upper Limit Switch", () -> upperLimitSwitchIsPressed(), null);
    builder.addBooleanProperty("Lower Limit Switch", () -> lowerLimitSwitchIsPressed(), null);
  }
}
