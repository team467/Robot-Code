package frc.robot.drive;

import com.revrobotics.CANSparkMaxLowLevel;

public class SpeedControllerFactory {
    public static SpeedControllerEncoder create(int motorID, MotorType type) {
        switch (type) {
            case TALON_SRX:
                return new TalonController(motorID);

            case SPARK_MAX_BRUSHED:
                return new SparkMaxController(motorID, CANSparkMaxLowLevel.MotorType.kBrushed);

            case SPARK_MAX_BRUSHLESS:
                return new SparkMaxController(motorID, CANSparkMaxLowLevel.MotorType.kBrushless);
        }

        return null;
    }
}
