package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.logging.RobotLogManager;
import frc.robot.motors.MotorControllerEncoder;
import frc.robot.motors.MotorControllerFactory;
import frc.robot.motors.MotorType;

import org.apache.logging.log4j.Logger;

public class Indexer2022 extends SubsystemBase {
    private static final Logger LOGGER = RobotLogManager.getMainLogger(Indexer2022.class.getName());

    private MotorControllerEncoder indexer;

    private boolean isReversed = false;

    public Indexer2022() {
        super();
        indexer = MotorControllerFactory.create(RobotConstants.get().indexer2022MotorID(), MotorType.TALON_SRX);
    }

    public void setIsReversed(boolean isReversed) {
        this.isReversed = isReversed;
    }

    public void indexerSlow() {
        //MAKE SURE SPEED IS PRETTY SLOW, apporximately 20%
        LOGGER.info("Starting indexer slowly, setting speed to " + RobotConstants.get().indexer2022SlowInSpeed());
        //indexer.set(RobotConstants.get().indexer2022SlowInSpeed());

    }

    public void indexerFast() {
        LOGGER.info("Starting indexer quickly, setting speed to " + RobotConstants.get().indexer2022InSpeed());
        //indexer.set(RobotConstants.get().indexer2022InSpeed());
    }

    public void indexerBackward() {
        LOGGER.info("Reversing indexer, setting speed to " + RobotConstants.get().indexer2022OutSpeed());
        //indexer.set(-RobotConstants.get().indexer2022OutSpeed());
    }

    public void indexerStop() {
        LOGGER.info("Stopping indexer, setting speed to 0");
        //indexer.set(0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("Indexer Position", () -> indexer.getPosition(), null);
        builder.addDoubleProperty("Indexer Velocity", () -> indexer.getVelocity(), null);
    }

}
