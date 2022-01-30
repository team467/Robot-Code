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

// down, up, enable, stop
public class Climber2022 extends SubsystemBase {

    private static final Logger LOGGER = RobotLogManager.getMainLogger(Climber2022.class.getName());


    //private MotorControllerEncoder climberMotor = MotorControllerFactory.create(RobotConstants.get().climber2022MotorID(), MotorType.);
    private boolean enabled = false;

    public Climber2022() {
        super();

        //climberMotor.setInverted(RobotConstants.get().climber) what
    }

    public void enable() {
        enabled = true;
        LOGGER.info("Climber enabled");
    }

    public void disable() {
        enabled = false;
        stop();
        LOGGER.info("Climber disabled");
    }

    public boolean isEnabled() {
        return enabled;
    }

    public void up() {
        if (enabled) {
            LOGGER.info("Climbing Up!");
        } else {
            LOGGER.info("Climb up does not work when disabled!");
        }
    }

    public void down() {
        if (enabled) {
            LOGGER.info("Climbing down");
        } else {
            LOGGER.info("Climb up doesn't work when disabled");
        }
    }

    public void stop () {
        //climberMotor.set(0);
        LOGGER.info("Climber stopped");
    }
}

