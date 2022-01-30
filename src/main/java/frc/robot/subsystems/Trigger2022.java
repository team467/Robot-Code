package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.logging.RobotLogManager;
import frc.robot.motors.MotorControllerEncoder;
import frc.robot.motors.MotorControllerFactory;
import frc.robot.motors.MotorType;

import org.apache.logging.log4j.Logger;

public class Trigger2022 extends SubsystemBase {
    private static final Logger LOGGER = RobotLogManager.getMainLogger(Trigger2022.class.getName());

    private MotorControllerEncoder triggerMotor;

    public Trigger2022() {
        super();
        triggerMotor = MotorControllerFactory.create(RobotConstants.get().trigger2022MotorID(), MotorType.TALON_SRX);
    }

    public void idle() {
        //MAKE SURE SPEED IS PRETTY SLOW, apporximately 20%
        LOGGER.debug("Starting indexer slowly, setting speed to " + RobotConstants.get().trigger2022IdleSpeed());
        triggerMotor.set(RobotConstants.get().trigger2022IdleSpeed());

    }

    public void forward() {
        LOGGER.debug("Starting indexer quickly, setting speed to " + RobotConstants.get().trigger2022InSpeed());
        triggerMotor.set(RobotConstants.get().trigger2022InSpeed());
    }

    public void backward() {
        LOGGER.debug("Reversing indexer, setting speed to " + RobotConstants.get().trigger2022OutSpeed());
        triggerMotor.set(-RobotConstants.get().trigger2022OutSpeed());
    }

    public void stop() {
        LOGGER.debug("Stopping indexer, setting speed to 0");
        triggerMotor.set(0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("Indexer Position", () -> triggerMotor.getPosition(), null);
        builder.addDoubleProperty("Indexer Velocity", () -> triggerMotor.getVelocity(), null);
    }

}
