package frc.robot.drive;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class MotorSpeedController implements SpeedController {
    SpeedController speedController;
    MotorType motorType;

    WPI_TalonSRX talon;
    CANSparkMax sparkMax;

    public MotorSpeedController (int channel, MotorType motorType) {
        this.motorType = motorType;
        
        switch (motorType) {
            case TALON_SRX:
                this.talon = new WPI_TalonSRX(channel);
                this.speedController = this.talon;
                break;
            case SPARK_MAX_BRUSHED:
                this.sparkMax = new CANSparkMax(channel, CANSparkMaxLowLevel.MotorType.kBrushed);
                this.speedController = this.sparkMax;
                break;
            case SPARK_MAX_BRUSHLESS:
                this.sparkMax = new CANSparkMax(channel, CANSparkMaxLowLevel.MotorType.kBrushless);
                this.speedController = this.sparkMax;
                break;
            default:
                break;
        }
    }

    @Override
    public void pidWrite(double output) {
        this.speedController.pidWrite(output);
    }

    @Override
    public void set(double speed) {
        this.speedController.set(speed);        
    }

    @Override
    public double get() {
        return this.speedController.get();
    }

    @Override
    public void setInverted(boolean isInverted) {
        this.speedController.setInverted(isInverted);        
    }

    @Override
    public boolean getInverted() {
        return this.speedController.getInverted();
    }

    @Override
    public void disable() {
        this.speedController.disable();
    }

    @Override
    public void stopMotor() {
        this.speedController.stopMotor();
    }

    public double getPosition() {
        double position = 0;
        switch (this.motorType) {
            case TALON_SRX:
                position = talon.getSelectedSensorPosition();
                break;
        
            case SPARK_MAX_BRUSHED:
            case SPARK_MAX_BRUSHLESS:
                position = sparkMax.getEncoder().getPosition();
                break;
            default:
                break;
        }
        return position;
    }

    public double getVelocity() {
        double velocity = 0;
        switch (this.motorType) {
            case TALON_SRX:
                velocity = talon.getSelectedSensorVelocity();
                break;
        
            case SPARK_MAX_BRUSHED:
            case SPARK_MAX_BRUSHLESS:
                velocity = sparkMax.getEncoder().getVelocity();
                break;
            default:
                break;
        }
        return velocity;
    }
}
