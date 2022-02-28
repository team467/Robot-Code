package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;
import frc.robot.RobotConstants;
import frc.robot.commands.Climber2022SetLeftSpeed;
import frc.robot.commands.Climber2022SetRightSpeed;
import frc.robot.commands.Climber2022StopCMD;
import frc.robot.logging.RobotLogManager;
import frc.robot.motors.MotorControllerEncoder;
import frc.robot.motors.MotorControllerFactory;
import frc.robot.motors.MotorType;

import frc.robot.tuning.SubsystemTuner;
import java.util.Map;
import org.apache.logging.log4j.Logger;

public class Climber2022 extends SubsystemTuner {

  // TODO make climber into state space and use real position control, after granite state
  private final MotorControllerEncoder climberMotorLeft = MotorControllerFactory.create(
      RobotConstants.get().climber2022LeftMotorId(), MotorType.SPARK_MAX_BRUSHLESS);
  private final MotorControllerEncoder climberMotorRight = MotorControllerFactory.create(
      RobotConstants.get().climber2022RightMotorId(), MotorType.SPARK_MAX_BRUSHLESS);
  private final Relay climberLock = new Relay(RobotConstants.get().climber2022SolenoidChannel());

  private boolean enabled = false;
  private static final Logger LOGGER = RobotLogManager.getMainLogger(Climber2022.class.getName());


  public Climber2022() {
    super();

    climberMotorLeft.setInverted(RobotConstants.get().climber2022LeftMotorInverted());
    climberMotorRight.setInverted(RobotConstants.get().climber2022RightMotorInverted());
    climberLock.set(Value.kOff);
  }

  public void enable() {
    climberLock.set(Value.kReverse);
    enabled = true;
  }

  public void disable() {
    climberLock.set(Value.kOff);
    stop();
    enabled = false;
  }

  public boolean isEnabled() {
    return enabled;
  }

  public void upLeft() {
    if (enabled) {
      if (climberMotorLeft.getPosition() < RobotConstants.get().climber2022LeftUpperLimit()) {
        climberMotorLeft.set(RobotConstants.get().climber2022UpSpeed());
      } else {
        stopLeft();
      }
    }
  }

  public void upRight() {
    if (enabled) {
      if (climberMotorRight.getPosition() < RobotConstants.get().climber2022RightUpperLimit()) {
        climberMotorRight.set(RobotConstants.get().climber2022UpSpeed());
      } else {
        stopRight();
      }
    }
  }

  public void up() {
    upLeft();
    upRight();
  }

  public void downSafeLeft() {
    if (enabled) {
      if (climberMotorLeft.getPosition() > RobotConstants.get().climber2022LeftLowerLimit()) {
        climberMotorLeft.set(-RobotConstants.get().climber2022DownSpeed());
      } else {
        stopLeft();
      }
    }
  }

  public void downSafeRight() {
    if (enabled) {
      if (climberMotorRight.getPosition() > RobotConstants.get().climber2022RightLowerLimit()) {
        climberMotorRight.set(-RobotConstants.get().climber2022DownSpeed());
      } else {
        stopRight();
      }
    }
  }

  public void downSafe() {
    downSafeLeft();
    downSafeRight();
  }

  public void downFullLeft() {
    if (enabled) {
      if (climberMotorLeft.getPosition() > 2) {
        climberMotorLeft.set(-RobotConstants.get().climber2022DownSpeed());
      } else {
        stopLeft();
      }
    }
  }

  public void downFullRight() {
    if (enabled) {
      if (climberMotorRight.getPosition() > 2) {
        climberMotorRight.set(-RobotConstants.get().climber2022DownSpeed());
      } else {
        stopRight();
      }
    }
  }

  public void downFull() {
    downFullLeft();
    downFullRight();
  }

  public void stopLeft() {
    climberMotorLeft.set(0);
  }

  public void stopRight() {
    climberMotorRight.set(0);
  }

  public void stop() {
    stopLeft();
    stopRight();
  }

  public void setLeftSpeed(double speed) {
    climberMotorLeft.set(speed);
  }

  public void setRightSpeed(double speed) {
    climberMotorRight.set(speed);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.addDoubleProperty("Left Climber Position", climberMotorLeft::getPosition, null);
    builder.addDoubleProperty("Left Climber Velocity", climberMotorLeft::getVelocity, null);

    builder.addDoubleProperty("Right Climber Position", climberMotorRight::getPosition, null);
    builder.addDoubleProperty("Right Climber Velocity", climberMotorRight::getVelocity, null);
  }

  @Override
  public void initializeTunerNetworkTables(ShuffleboardTab tab) {
    addEntry("leftSpeed",
        tab.add("Left Speed", 0).withWidget(BuiltInWidgets.kNumberSlider).withSize(2, 1)
            .withPosition(3, 1).withProperties(
                Map.of("min", -0.3, "max", 0.3)).getEntry());
    addEntry("runLeft",
        tab.add("Run Left", false).withWidget(BuiltInWidgets.kToggleButton).withSize(2, 1)
            .withPosition(3, 2).getEntry());

    addEntry("rightSpeed",
        tab.add("Right Speed", 0).withWidget(BuiltInWidgets.kNumberSlider).withSize(2, 1)
            .withPosition(5, 1).withProperties(
                Map.of("min", -0.3, "max", 0.3)).getEntry());
    addEntry("runRight",
        tab.add("Run Right", false).withWidget(BuiltInWidgets.kToggleButton).withSize(2, 1)
            .withPosition(5, 2).getEntry());
  }

  @Override
  public void initializeTuner() {
    getEntry("leftSpeed").setDouble(0);
    getEntry("runLeft").setBoolean(false);

    getEntry("rightSpeed").setDouble(0);
    getEntry("runRight").setBoolean(false);

    this.setDefaultCommand(new Climber2022StopCMD(this));
    new NetworkButton(getEntry("runLeft")).whileActiveContinuous(
        new Climber2022SetLeftSpeed(this,
            () -> getEntry("leftSpeed").getDouble(0)
        )
    ).whenActive(() -> {
      getEntry("runRight").setBoolean(false);
    });

    new NetworkButton(getEntry("runRight")).whileActiveContinuous(
        new Climber2022SetRightSpeed(this,
            () -> getEntry("rightSpeed").getDouble(0)
        )
    ).whenActive(() -> {
      getEntry("runLeft").setBoolean(false);
    });
  }
}

