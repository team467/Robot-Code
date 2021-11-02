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
import frc.robot.drive.SparkMaxController;
import frc.robot.drive.SpeedControllerEncoder;
import frc.robot.drive.TalonController;

public class Drivetrain extends SubsystemBase {
    SpeedControllerGroup leftMotorGroup;
    SpeedControllerEncoder leftMotorLeader;
    SpeedControllerEncoder leftMotorFollower = null;

    SpeedControllerGroup rightMotorGroup;
    SpeedControllerEncoder rightMotorLeader;
    SpeedControllerEncoder rightMotorFollower = null;
    
    DifferentialDrive diffDrive;

    public Drivetrain() {
        super();

        switch (Constants.DRIVE_MOTOR_LEFT_LEADER_TYPE) {
            case TALON_SRX:
                leftMotorLeader = new TalonController(Constants.DRIVE_MOTOR_LEFT_LEADER_ID);
                break;

            case SPARK_MAX_BRUSHED:
                leftMotorLeader = new SparkMaxController(Constants.DRIVE_MOTOR_LEFT_LEADER_ID, MotorType.kBrushed);
                break;

            case SPARK_MAX_BRUSHLESS:
                leftMotorLeader = new SparkMaxController(Constants.DRIVE_MOTOR_LEFT_LEADER_ID, MotorType.kBrushless);
                break;
        }

        switch (Constants.DRIVE_MOTOR_RIGHT_LEADER_TYPE) {
            case TALON_SRX:
                rightMotorLeader = new TalonController(Constants.DRIVE_MOTOR_RIGHT_LEADER_ID);
                break;

            case SPARK_MAX_BRUSHED:
                rightMotorLeader = new SparkMaxController(Constants.DRIVE_MOTOR_RIGHT_LEADER_ID, MotorType.kBrushed);
                break;

            case SPARK_MAX_BRUSHLESS:
                rightMotorLeader = new SparkMaxController(Constants.DRIVE_MOTOR_RIGHT_LEADER_ID, MotorType.kBrushless);
                break;
        }

        leftMotorLeader.setInverted(Constants.DRIVE_MOTOR_LEFT_LEADER_INVERTED);
        rightMotorLeader.setInverted(Constants.DRIVE_MOTOR_RIGHT_LEADER_INVERTED);

        if (Constants.DRIVE_DUAL_MOTORS) {
            switch (Constants.DRIVE_MOTOR_LEFT_FOLLOWER_TYPE) {
                case TALON_SRX:
                    leftMotorFollower = new TalonController(Constants.DRIVE_MOTOR_LEFT_FOLLOWER_ID);
                    break;
    
                case SPARK_MAX_BRUSHED:
                    leftMotorFollower = new SparkMaxController(Constants.DRIVE_MOTOR_LEFT_FOLLOWER_ID, MotorType.kBrushed);
                    break;
    
                case SPARK_MAX_BRUSHLESS:
                    leftMotorFollower = new SparkMaxController(Constants.DRIVE_MOTOR_LEFT_FOLLOWER_ID, MotorType.kBrushless);
                    break;
            }
    
            switch (Constants.DRIVE_MOTOR_RIGHT_FOLLOWER_TYPE) {
                case TALON_SRX:
                    rightMotorFollower = new TalonController(Constants.DRIVE_MOTOR_RIGHT_FOLLOWER_ID);
                    break;
    
                case SPARK_MAX_BRUSHED:
                    rightMotorFollower = new SparkMaxController(Constants.DRIVE_MOTOR_RIGHT_FOLLOWER_ID, MotorType.kBrushed);
                    break;
    
                case SPARK_MAX_BRUSHLESS:
                    rightMotorFollower = new SparkMaxController(Constants.DRIVE_MOTOR_RIGHT_FOLLOWER_ID, MotorType.kBrushless);
                    break;
            }
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
