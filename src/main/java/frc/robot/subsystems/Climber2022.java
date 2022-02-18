package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.logging.RobotLogManager;
import frc.robot.motors.MotorControllerEncoder;
import frc.robot.motors.MotorControllerFactory;
import frc.robot.motors.MotorType;

import org.apache.logging.log4j.Logger;

// down, up, enable, stop
public class Climber2022 extends SubsystemBase {
//Okay, so basically the right motor's id is 6 and the left motor's id is 11 as of now. The "front" of the robot is the side where the top of the llamma neck opens up for now, but apparently drivers like it to be the other way around (not too sure about this, it's just what some mechanical kid said). However, from a building perspective the way the llama neck's opening is facing is the front.

    private MotorControllerEncoder climberMotorRight =MotorControllerFactory.create(RobotConstants.get().climber2022MotorIdRight(),MotorType.SPARK_MAX_BRUSHLESS);
    private MotorControllerEncoder climberMotorLeft =MotorControllerFactory.create(RobotConstants.get().climber2022MotorIdLeft(),MotorType.SPARK_MAX_BRUSHLESS);
    private Relay climberLock =new Relay(RobotConstants.get().climber2022SolenoidChannel());

    private boolean enabled = false;
    private static final Logger LOGGER = RobotLogManager.getMainLogger(Climber2022.class.getName());


    public Climber2022() {
        super();

        climberMotorRight.setInverted(RobotConstants.get().climber2022MotorInvertedRight());
        climberMotorLeft.setInverted(RobotConstants.get().climber2022MotorInvertedLeft());
        climberLock.setDirection(Direction.kReverse);
    }

    public void enable() {
        climberLock.setDirection(Direction.kForward);
        enabled = true;
    }

    public void disable() {
        climberLock.setDirection(Direction.kReverse);
        stop();
        enabled = false;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public void up() {
        if(this.isEnabled()) {
            climberMotorRight.set(RobotConstants.get().climber2022UpSpeed());
            climberMotorLeft.set(RobotConstants.get().climber2022UpSpeed());
        }
    }

    public void down() {
        if (this.isEnabled()){
            climberMotorRight.set(-RobotConstants.get().climber2022DownSpeed());
            climberMotorLeft.set(-RobotConstants.get().climber2022DownSpeed());
        }
    }

    public void stop() {
        climberMotorRight.set(0);
        climberMotorLeft.set(0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("Climber 1 Position", () -> climberMotorRight.getPosition(),null);
        builder.addDoubleProperty("Climber 1 Velocity", () -> climberMotorRight.getVelocity(), null);

        builder.addDoubleProperty("Climber 2 Position", () -> climberMotorLeft.getPosition(),null);
        builder.addDoubleProperty("Climber 2 Velocity", () -> climberMotorLeft.getVelocity(), null);
    }
}

