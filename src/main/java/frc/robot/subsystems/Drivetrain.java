package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.drive.MotorSpeedController;

public class Drivetrain extends SubsystemBase {
    MotorSpeedController motor1 = new MotorSpeedController(Constants.DRIVE_MOTOR_1_ID, Constants.DRIVE_MOTOR_1_TYPE);
    MotorSpeedController motor2 = new MotorSpeedController(Constants.DRIVE_MOTOR_2_ID, Constants.DRIVE_MOTOR_2_TYPE);
    
    DifferentialDrive diffDrive = new DifferentialDrive(motor1, motor2);

    public Drivetrain() {
        super();
    }

    public void arcadeDrive(double speed, double rotation) {
        diffDrive.arcadeDrive(speed, rotation);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty(".motor_1_position", () -> motor1.getPosition(), null);
        builder.addDoubleProperty(".motor_1_speed", () -> motor2.getVelocity(), null);
        builder.addDoubleProperty(".motor_2_position", () -> motor2.getPosition(), null);
        builder.addDoubleProperty(".motor_2_speed", () -> motor2.getVelocity(), null);
    }
}
