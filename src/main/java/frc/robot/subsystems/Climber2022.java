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
    private MotorControllerEncoder climberMotor = MotorControllerFactory.create(RobotConstants.get().climber2020MotorID(), MotorType.//);
    private boolean enabled = false;

    public Climber2022() {
        super();

        //climberMotor.setInverted(RobotConstants.get().climber) what
    }

    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public void up() {
        //
    }

    public void down() {
        //
    }

    public void stop () {
        climberMotor.set(0);
    }
}

