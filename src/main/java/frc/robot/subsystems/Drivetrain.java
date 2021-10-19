package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
    // CANSparkMax spark1 = new CANSparkMax(Constants.SPARK_1_ID, MotorType.kBrushless);
    // CANSparkMax spark2 = null;
    TalonSRX talon1 = new TalonSRX(11);
    // TalonSRX talon2 = new TalonSRX(12);
    // SpeedControllerGroup talon1s = new SpeedController(talon1);
    // CANSparkMax spark2 = new CANSparkMax(Constants.SPARK_2_ID, MotorType.kBrushless);
    // DifferentialDrive diffDrive = new DifferentialDrive(talon1, talon2);

    public Drivetrain() {
        super();
    }

    public void arcadeDrive(double speed, double rotation) {
        // diffDrive.arcadeDrive(speed, rotation);
        talon1.set(TalonSRXControlMode.PercentOutput, speed);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        // builder.addDoubleProperty(".spark_1_position", () -> spark1.getEncoder().getPosition(), null);
        // builder.addDoubleProperty(".spark_1_speed", () -> spark1.getEncoder().getVelocity(), null);
    }
}
