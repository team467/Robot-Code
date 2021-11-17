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
import frc.robot.motors.SparkMaxController;
import frc.robot.motors.SpeedControllerEncoder;
import frc.robot.motors.SpeedControllerFactory;
import frc.robot.motors.TalonController;

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

        leftMotorLeader = SpeedControllerFactory.create(RobotConstants.get().driveMotorLeftLeaderId(), RobotConstants.get().driveMotorType());
        rightMotorLeader = SpeedControllerFactory.create(RobotConstants.get().driveMotorRightLeaderId(), RobotConstants.get().driveMotorType());

        leftMotorLeader.setInverted(RobotConstants.get().driveMotorLeftLeaderInverted());
        rightMotorLeader.setInverted(RobotConstants.get().driveMotorRightLeaderInverted());

        if (RobotConstants.get().driveDualMotors()) {
            leftMotorFollower = SpeedControllerFactory.create(RobotConstants.get().driveMotorLeftFollowerId(), RobotConstants.get().driveMotorType());
            rightMotorFollower = SpeedControllerFactory.create(RobotConstants.get().driveMotorRightFollowerId(), RobotConstants.get().driveMotorType());
            
            leftMotorFollower.setInverted(RobotConstants.get().driveMotorLeftFollowerInverted());
            rightMotorFollower.setInverted(RobotConstants.get().driveMotorRightFollowerInverted());

            leftMotorGroup = new SpeedControllerGroup(leftMotorLeader, leftMotorFollower);
            rightMotorGroup = new SpeedControllerGroup(rightMotorLeader, rightMotorFollower);
        } else {
            leftMotorGroup = new SpeedControllerGroup(leftMotorLeader);
            rightMotorGroup = new SpeedControllerGroup(rightMotorLeader);
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
