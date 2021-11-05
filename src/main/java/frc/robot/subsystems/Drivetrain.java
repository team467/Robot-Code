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
import frc.robot.RobotConstants;
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

        switch (RobotConstants.get().driveMotorLeftLeaderType()) {
            case TALON_SRX:
                leftMotorLeader = new TalonController(RobotConstants.get().driveMotorLeftLeaderId());
                break;

            case SPARK_MAX_BRUSHED:
                leftMotorLeader = new SparkMaxController(RobotConstants.get().driveMotorLeftLeaderId(), MotorType.kBrushed);
                break;

            case SPARK_MAX_BRUSHLESS:
                leftMotorLeader = new SparkMaxController(RobotConstants.get().driveMotorLeftLeaderId(), MotorType.kBrushless);
                break;
        }

        switch (RobotConstants.get().driveMotorRightLeaderType()) {
            case TALON_SRX:
                rightMotorLeader = new TalonController(RobotConstants.get().driveMotorRightLeaderId());
                break;

            case SPARK_MAX_BRUSHED:
                rightMotorLeader = new SparkMaxController(RobotConstants.get().driveMotorRightLeaderId(), MotorType.kBrushed);
                break;

            case SPARK_MAX_BRUSHLESS:
                rightMotorLeader = new SparkMaxController(RobotConstants.get().driveMotorRightLeaderId(), MotorType.kBrushless);
                break;
        }

        leftMotorLeader.setInverted(RobotConstants.get().driveMotorLeftLeaderInverted());
        rightMotorLeader.setInverted(RobotConstants.get().driveMotorRightLeaderInverted());

        leftMotorGroup = new SpeedControllerGroup(leftMotorLeader);
        rightMotorGroup = new SpeedControllerGroup(rightMotorLeader);

        if (RobotConstants.get().driveDualMotors()) {
            switch (RobotConstants.get().driveMotorLeftFollowerType()) {
                case TALON_SRX:
                    leftMotorFollower = new TalonController(RobotConstants.get().driveMotorLeftFollowerId());
                    break;
    
                case SPARK_MAX_BRUSHED:
                    leftMotorFollower = new SparkMaxController(RobotConstants.get().driveMotorLeftFollowerId(), MotorType.kBrushed);
                    break;
    
                case SPARK_MAX_BRUSHLESS:
                    leftMotorFollower = new SparkMaxController(RobotConstants.get().driveMotorLeftFollowerId(), MotorType.kBrushless);
                    break;
            }
    
            switch (RobotConstants.get().driveMotorRightFollowerType()) {
                case TALON_SRX:
                    rightMotorFollower = new TalonController(RobotConstants.get().driveMotorRightFollowerId());
                    break;
    
                case SPARK_MAX_BRUSHED:
                    rightMotorFollower = new SparkMaxController(RobotConstants.get().driveMotorRightFollowerId(), MotorType.kBrushed);
                    break;
    
                case SPARK_MAX_BRUSHLESS:
                    rightMotorFollower = new SparkMaxController(RobotConstants.get().driveMotorRightFollowerId(), MotorType.kBrushless);
                    break;
            }
            leftMotorFollower.setInverted(RobotConstants.get().driveMotorLeftFollowerInverted());
            rightMotorFollower.setInverted(RobotConstants.get().driveMotorRightFollowerInverted());

            leftMotorGroup = new SpeedControllerGroup(leftMotorLeader, leftMotorFollower);
            rightMotorGroup = new SpeedControllerGroup(rightMotorLeader, rightMotorFollower);
        }

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
