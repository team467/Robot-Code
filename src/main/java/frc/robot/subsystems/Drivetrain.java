package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.motors.SparkMaxController;
import frc.robot.motors.MotorControllerEncoder;
import frc.robot.motors.MotorControllerFactory;
import frc.robot.motors.MotorType;

public class Drivetrain extends SubsystemBase {
    MotorControllerGroup leftMotorGroup;
    MotorControllerEncoder leftMotorLeader;
    MotorControllerEncoder leftMotorFollower = null;

    MotorControllerGroup rightMotorGroup;
    MotorControllerEncoder rightMotorLeader;
    MotorControllerEncoder rightMotorFollower = null;

    SimpleMotorFeedforward driveFF;
    PIDController leftDrivePID;
    PIDController rightDrivePID;

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

        leftMotorLeader.resetPosition();
        rightMotorLeader.resetPosition();

        leftMotorLeader.setUnitsPerRotation(RobotConstants.get().driveMetersPerRotation());
        rightMotorLeader.setUnitsPerRotation(RobotConstants.get().driveMetersPerRotation());

        if (RobotConstants.get().driveMotorType() == MotorType.SPARK_MAX_BRUSHLESS) {
            ((SparkMaxController) leftMotorLeader).setIdleMode(IdleMode.kCoast);
            ((SparkMaxController) rightMotorLeader).setIdleMode(IdleMode.kCoast);
        }

        if (RobotConstants.get().driveDualMotors()) {
            leftMotorFollower = MotorControllerFactory.create(RobotConstants.get().driveMotorLeftFollowerId(),
                    RobotConstants.get().driveMotorType());
            rightMotorFollower = MotorControllerFactory.create(RobotConstants.get().driveMotorRightFollowerId(),
                    RobotConstants.get().driveMotorType());

            leftMotorFollower.setInverted(RobotConstants.get().driveMotorLeftFollowerInverted());
            rightMotorFollower.setInverted(RobotConstants.get().driveMotorRightFollowerInverted());

            leftMotorFollower.resetPosition();
            rightMotorFollower.resetPosition();

            leftMotorFollower.setUnitsPerRotation(RobotConstants.get().driveMetersPerRotation());
            rightMotorFollower.setUnitsPerRotation(RobotConstants.get().driveMetersPerRotation());

            leftMotorGroup = new MotorControllerGroup(leftMotorLeader, leftMotorFollower);
            rightMotorGroup = new MotorControllerGroup(rightMotorLeader, rightMotorFollower);

            if (RobotConstants.get().driveMotorType() == MotorType.SPARK_MAX_BRUSHLESS) {
                ((SparkMaxController) leftMotorFollower).setIdleMode(IdleMode.kCoast);
                ((SparkMaxController) rightMotorFollower).setIdleMode(IdleMode.kCoast);
            }
        } else {
            leftMotorGroup = new MotorControllerGroup(leftMotorLeader);
            rightMotorGroup = new MotorControllerGroup(rightMotorLeader);
        }

        if (RobotConstants.get().driveUseVelocity()) {
            driveFF = RobotConstants.get().driveDriveFF().getFeedforward();

            if (RobotConstants.get().driveUsePID()) {
                leftDrivePID = RobotConstants.get().driveDriveVelocityPID().getPIDController();
                rightDrivePID = RobotConstants.get().driveDriveVelocityPID().getPIDController();
            }
        }

        diffDrive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);
    }

    public void arcadeDrive(double speed, double rotation) {
        if (RobotConstants.get().driveUseVelocity()) {
            DifferentialDrive.WheelSpeeds speeds = DifferentialDrive.arcadeDriveIK(speed, rotation, true);
            double leftVelocity =  speeds.left * RobotConstants.get().driveMaxVelocity();
            double rightVelocity =  speeds.right * RobotConstants.get().driveMaxVelocity();
            double leftVoltage = driveFF.calculate(leftVelocity);
            double rightVoltage = driveFF.calculate(rightVelocity);

            if (RobotConstants.get().driveUsePID()) {
                leftVoltage += leftDrivePID.calculate(getLeftVelocity(), leftVelocity);
                rightVoltage += rightDrivePID.calculate(getRightVelocity(), rightVelocity);
            }

            leftMotorGroup.setVoltage(leftVoltage);
            rightMotorGroup.setVoltage(rightVoltage);
        } else {
            diffDrive.arcadeDrive(speed, rotation);
        }
    }

    public void curvatureDrive(double speed, double rotation, boolean turnInPlace) {
        diffDrive.curvatureDrive(speed, rotation, turnInPlace);
    }

    public void curvatureDrive(double speed, double rotation) {
        curvatureDrive(speed, rotation, true);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMotorGroup.setVoltage(leftVolts);
        rightMotorGroup.setVoltage(rightVolts);
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

    public void resetLeftPID() {
        leftDrivePID.reset();
    }

    public void resetRightPID() {
        rightDrivePID.reset();
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
