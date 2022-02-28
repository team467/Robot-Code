package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.logging.RobotLogManager;
import frc.robot.motors.MotorControllerEncoder;
import frc.robot.motors.MotorControllerFactory;
import frc.robot.motors.MotorType;

import org.apache.logging.log4j.Logger;

public class Climber2022 extends SubsystemBase {
    private final MotorControllerEncoder climberMotorLeft = MotorControllerFactory.create(RobotConstants.get().climber2022LeftMotorId(),MotorType.SPARK_MAX_BRUSHLESS);
    private final MotorControllerEncoder climberMotorRight = MotorControllerFactory.create(RobotConstants.get().climber2022RightMotorId(),MotorType.SPARK_MAX_BRUSHLESS);
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

    public void up() {
        if(enabled) {
            if (climberMotorLeft.getPosition() < RobotConstants.get().climber2022LeftUpperLimit()) {
                climberMotorLeft.set(RobotConstants.get().climber2022UpSpeed());
            } else {
                climberMotorLeft.set(0);
            }

            if (climberMotorRight.getPosition() < RobotConstants.get().climber2022RightUpperLimit()) {
                climberMotorRight.set(RobotConstants.get().climber2022UpSpeed());
            } else {
                climberMotorRight.set(0);
            }

        }
    }

    public void downSafe() {
        if (enabled){
            if (climberMotorLeft.getPosition() > RobotConstants.get().climber2022LeftLowerLimit()) {
                climberMotorLeft.set(-RobotConstants.get().climber2022DownSpeed());
            } else {
                climberMotorLeft.set(0);
            }
            if (climberMotorRight.getPosition() > RobotConstants.get().climber2022RightLowerLimit()) {
                climberMotorRight.set(-RobotConstants.get().climber2022DownSpeed());
            } else {
                climberMotorRight.set(0);
            }

        }
    }

    public void downFull() {
        if (enabled){
            if (climberMotorLeft.getPosition() > 2) {
                climberMotorLeft.set(-RobotConstants.get().climber2022DownSpeed());
            } else {
                climberMotorLeft.set(0);
            }
            if (climberMotorRight.getPosition() > 2) {
                climberMotorRight.set(-RobotConstants.get().climber2022DownSpeed());
            } else {
                climberMotorRight.set(0);
            }

        }
    }

    public void stop() {
        climberMotorLeft.set(0);
        climberMotorRight.set(0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("Left Climber Position", climberMotorLeft::getPosition,null);
        builder.addDoubleProperty("Left Climber Velocity", climberMotorLeft::getVelocity, null);

        builder.addDoubleProperty("Right Climber Position", climberMotorRight::getPosition,null);
        builder.addDoubleProperty("Right Climber Velocity", climberMotorRight::getVelocity, null);
    }
}

