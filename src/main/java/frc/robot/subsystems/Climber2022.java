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
    private MotorControllerEncoder climberMotor =MotorControllerFactory.create(RobotConstants.get().climber2022MotorId(),MotorType.SPARK_MAX_BRUSHLESS);
    private boolean enabled = false;
    private static final Logger LOGGER = RobotLogManager.getMainLogger(Climber2022.class.getName());


    public Climber2022() {
        super();

        climberMotor.setInverted(RobotConstants.get().climber2022MotorInverted());
    }

    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
        stop();
    }

    public boolean isEnabled() {
        return enabled;
    }

    public void up() {
        if(this.isEnabled()) {
            climberMotor.set(RobotConstants.get().climber2022UpSpeed());
        }
    }

    public void down() {
        if (this.isEnabled()){
            climberMotor.set(RobotConstants.get().climber2022DownSpeed());
        }
    }

    public void stop () {
        climberMotor.set(0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("Climber Position", () -> climberMotor.getPosition(),null);
        builder.addDoubleProperty("Climber Velocity", () -> climberMotor.getVelocity(), null);
    }
}

