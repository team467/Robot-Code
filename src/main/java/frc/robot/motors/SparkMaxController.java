package frc.robot.motors;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SparkMaxController implements SpeedControllerEncoder {
    private CANSparkMax spark;
    private CANPIDController pid;
    
    public SparkMaxController(int id, MotorType motorType) {
        spark = new CANSparkMax(id, motorType);
    }

    @Override
    public void set(double speed) {
        spark.set(speed);
    }

    @Override
    public double get() {
        return spark.get();
    }

    @Override
    public void setInverted(boolean isInverted) {
        spark.setInverted(isInverted);
    }

    @Override
    public boolean getInverted() {
        return spark.getInverted();
    }

    @Override
    public void disable() {
        spark.disable();
    }

    @Override
    public void stopMotor() {
        spark.stopMotor();
    }

    @Override
    public void pidWrite(double output) {
        spark.pidWrite(output);
    }

    @Override
    public void setP(double kP) {
        pid.setP(kP);
    }

    @Override
    public void setI(double kI) {
        pid.setI(kI);        
    }

    @Override
    public void setD(double kD) {
        pid.setD(kD);        
    }

    @Override
    public void setF(double kF) {
        pid.setFF(kF);        
    }
    
    @Override
    public void set(double value, ControlType controlType) {
        switch (controlType) {
        case Current:
            pid.setReference(value, com.revrobotics.ControlType.kCurrent);
            break;
        case PercentOutput:
            pid.setReference(value, com.revrobotics.ControlType.kDutyCycle);
            break;
        case Position:
            pid.setReference(value, com.revrobotics.ControlType.kPosition);
            break;
        case Velocity:
            pid.setReference(value, com.revrobotics.ControlType.kVelocity);
            break;
        }        
    }

    @Override
    public double getPosition() {
        return spark.getEncoder().getPosition();
    }

    @Override
    public double getVelocity() {
        return spark.getEncoder().getVelocity();
    }

    @Override
    public double getCurrent() {
        return spark.getOutputCurrent();
    }
}
