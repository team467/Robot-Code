package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.logging.RobotLogManager;
import frc.robot.motors.MotorControllerEncoder;
import frc.robot.motors.MotorControllerFactory;
import frc.robot.motors.MotorType;

import org.apache.logging.log4j.Logger;

public class Intake2022 extends SubsystemBase {
    private static final Logger LOGGER = RobotLogManager.getMainLogger(Intake2022.class.getName());

    private MotorControllerEncoder intake;

    private boolean isReversed = false;

    public Intake2022() {
        super();
        intake = MotorControllerFactory.create(RobotConstants.get().intake2022MotorID(), MotorType.TALON_SRX);
    }

    public void setIsReversed(boolean isReversed) {
        this.isReversed = isReversed;
    }

    public void intakeIn() {
        LOGGER.info("Starting intake");
        intake.set(RobotConstants.get().intake2022InSpeed());
    }

    public void intakeOut() {
        LOGGER.info("Reversing intake");
        intake.set(-RobotConstants.get().intake2022OutSpeed());
    }

    public void intakeStop() {
        LOGGER.info("Stopping intake");
        intake.set(0);
    }

}
