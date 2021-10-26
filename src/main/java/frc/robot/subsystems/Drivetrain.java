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
    SpeedControllerGroup leftMotorGroup;
    MotorSpeedController leftMotorLeader;
    MotorSpeedController leftMotorFollower = null;

    SpeedControllerGroup rightMotorGroup;
    MotorSpeedController rightMotorLeader;
    MotorSpeedController rightMotorFollower = null;
    
    DifferentialDrive diffDrive;

    public Drivetrain() {
        super();

        leftMotorLeader = new MotorSpeedController(Constants.DRIVE_MOTOR_LEFT_LEADER_ID, Constants.DRIVE_MOTOR_LEFT_LEADER_TYPE);
        rightMotorLeader = new MotorSpeedController(Constants.DRIVE_MOTOR_RIGHT_LEADER_ID, Constants.DRIVE_MOTOR_RIGHT_LEADER_TYPE);

        leftMotorLeader.setInverted(Constants.DRIVE_MOTOR_LEFT_LEADER_INVERTED);
        rightMotorLeader.setInverted(Constants.DRIVE_MOTOR_RIGHT_LEADER_INVERTED);

        if (Constants.DRIVE_DUAL_MOTORS) {
            leftMotorFollower = new MotorSpeedController(Constants.DRIVE_MOTOR_LEFT_FOLLOWER_ID, Constants.DRIVE_MOTOR_LEFT_FOLLOWER_TYPE);
            rightMotorFollower = new MotorSpeedController(Constants.DRIVE_MOTOR_RIGHT_FOLLOWER_ID, Constants.DRIVE_MOTOR_RIGHT_FOLLOWER_TYPE);
            
            leftMotorFollower.setInverted(Constants.DRIVE_MOTOR_LEFT_FOLLOWER_INVERTED);
            rightMotorFollower.setInverted(Constants.DRIVE_MOTOR_RIGHT_FOLLOWER_INVERTED);
        }

        leftMotorGroup = new SpeedControllerGroup(leftMotorLeader, leftMotorFollower);
        rightMotorGroup = new SpeedControllerGroup(rightMotorLeader, rightMotorFollower);

        diffDrive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);
    }

    public void arcadeDrive(double speed, double rotation) {
        diffDrive.arcadeDrive(speed, rotation);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty(".motor_left_position", () -> leftMotorLeader.getPosition(), null);
        builder.addDoubleProperty(".motor_left_speed", () -> leftMotorLeader.getVelocity(), null);
        builder.addDoubleProperty(".motor_right_position", () -> rightMotorLeader.getPosition(), null);
        builder.addDoubleProperty(".motor_right_speed", () -> rightMotorLeader.getVelocity(), null);
    }
}
