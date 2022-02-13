package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.motors.SparkMaxController;
import frc.robot.motors.FeedMotorControllerEncoderGroup;
import frc.robot.motors.MotorControllerEncoder;
import frc.robot.motors.MotorControllerFactory;
import frc.robot.motors.MotorType;

public class Drivetrain extends SubsystemBase {
    FeedMotorControllerEncoderGroup leftMotorGroup;
    MotorControllerEncoder leftMotorLeader;
    MotorControllerEncoder leftMotorFollower = null;

    FeedMotorControllerEncoderGroup rightMotorGroup;
    MotorControllerEncoder rightMotorLeader;
    MotorControllerEncoder rightMotorFollower = null;

    DifferentialDrive diffDrive;

    public Drivetrain() {
        super();

        leftMotorLeader = MotorControllerFactory.create(RobotConstants.get().driveMotorLeftLeaderId(),
                RobotConstants.get().driveMotorType());
        rightMotorLeader = MotorControllerFactory.create(RobotConstants.get().driveMotorRightLeaderId(),
                RobotConstants.get().driveMotorType());

        leftMotorLeader.setInverted(RobotConstants.get().driveMotorLeftLeaderInverted());
        // No longer auto inverted by diff drive, you must invert it
        rightMotorLeader.setInverted(RobotConstants.get().driveMotorRightLeaderInverted());

        leftMotorLeader.setUnitsPerRotation(RobotConstants.get().driveMetersPerRotation());
        rightMotorLeader.setUnitsPerRotation(RobotConstants.get().driveMetersPerRotation());

        if (RobotConstants.get().driveMotorType() == MotorType.SPARK_MAX_BRUSHLESS) {
            ((SparkMaxController) leftMotorLeader).setIdleMode(IdleMode.kBrake);
            ((SparkMaxController) rightMotorLeader).setIdleMode(IdleMode.kBrake);
        }

        if (RobotConstants.get().driveDualMotors()) {
            leftMotorFollower = MotorControllerFactory.create(RobotConstants.get().driveMotorLeftFollowerId(),
                    RobotConstants.get().driveMotorType());
            rightMotorFollower = MotorControllerFactory.create(RobotConstants.get().driveMotorRightFollowerId(),
                    RobotConstants.get().driveMotorType());

            leftMotorFollower.setInverted(RobotConstants.get().driveMotorLeftFollowerInverted());
            rightMotorFollower.setInverted(RobotConstants.get().driveMotorRightFollowerInverted());

            leftMotorFollower.setUnitsPerRotation(RobotConstants.get().driveMetersPerRotation());
            rightMotorFollower.setUnitsPerRotation(RobotConstants.get().driveMetersPerRotation());

            leftMotorGroup = new FeedMotorControllerEncoderGroup(leftMotorLeader, leftMotorFollower);
            rightMotorGroup = new FeedMotorControllerEncoderGroup(rightMotorLeader, rightMotorFollower);

            if (RobotConstants.get().driveMotorType() == MotorType.SPARK_MAX_BRUSHLESS) {
                ((SparkMaxController) leftMotorFollower).setIdleMode(IdleMode.kBrake);
                ((SparkMaxController) rightMotorFollower).setIdleMode(IdleMode.kBrake);
            }
        } else {
            leftMotorGroup = new FeedMotorControllerEncoderGroup(leftMotorLeader);
            rightMotorGroup = new FeedMotorControllerEncoderGroup(rightMotorLeader);
        }

        if (RobotConstants.get().driveUseVelocity()) {


            if (RobotConstants.get().driveUsePID()) {
                // TODO NEW PID
            }
        }

        diffDrive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);
    }

    public void arcadeDrive(double speed, double rotation) {
        diffDrive.arcadeDrive(speed, rotation);
    }

    public void curvatureDrive(double speed, double rotation, boolean turnInPlace) {
        diffDrive.curvatureDrive(speed, rotation, turnInPlace);
    }

    public void curvatureDrive(double speed, double rotation) {
        curvatureDrive(speed, rotation, true);
    }

    public void tankDriveVolts(double leftVolt, double rightVolt) {
        leftMotorGroup.setVoltage(leftVolt);
        rightMotorGroup.setVoltage(rightVolt);
    }

    public double getLeftPosition() {
        return leftMotorLeader.getPosition();
    }

    public double getRightPosition() {
        return rightMotorLeader.getPosition();
    }

    public double getLeftVelocity() {
        return leftMotorLeader.getVelocity();
    }

    public double getRightVelocity() {
        return rightMotorLeader.getVelocity();
    }

    public void resetRightPosition() {
        rightMotorLeader.resetPosition();
    }

    public void resetLeftPosition() {
        leftMotorLeader.resetPosition();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("Motor Left Position", () -> leftMotorLeader.getPosition(), null);
        builder.addDoubleProperty("Motor Left Velocity", () -> leftMotorLeader.getVelocity(), null);
        builder.addDoubleProperty("Motor Right Position", () -> rightMotorLeader.getPosition(), null);
        builder.addDoubleProperty("Motor Right Velocity", () -> rightMotorLeader.getVelocity(), null);
    }
}
