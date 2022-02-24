package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;
import frc.robot.RobotConstants;
import frc.robot.commands.ArcadeDriveCMD;
import frc.robot.commands.DrivetrainStopCMD;
import frc.robot.motors.SparkMaxController;
import frc.robot.motors.MotorControllerEncoder;
import frc.robot.motors.MotorControllerFactory;
import frc.robot.motors.MotorType;
import frc.robot.tuning.SubsystemTuner;

public class Drivetrain extends SubsystemTuner {
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

        leftMotorLeader.setUnitsPerRotation(Math.PI * RobotConstants.get().driveWheelDiameter() * RobotConstants.get().driveGearRatio().getRotationsPerInput());
        rightMotorLeader.setUnitsPerRotation(Math.PI * RobotConstants.get().driveWheelDiameter() * RobotConstants.get().driveGearRatio().getRotationsPerInput());

        if (RobotConstants.get().driveMotorType() == MotorType.SPARK_MAX_BRUSHLESS) {
            ((SparkMaxController) leftMotorLeader).setIdleMode(RobotConstants.get().driveIdleMode());
            ((SparkMaxController) rightMotorLeader).setIdleMode(RobotConstants.get().driveIdleMode());
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

            leftMotorFollower.setUnitsPerRotation(Math.PI * RobotConstants.get().driveWheelDiameter() * RobotConstants.get().driveGearRatio().getRotationsPerInput());
            rightMotorFollower.setUnitsPerRotation(Math.PI * RobotConstants.get().driveWheelDiameter() * RobotConstants.get().driveGearRatio().getRotationsPerInput());

            leftMotorGroup = new MotorControllerGroup(leftMotorLeader, leftMotorFollower);
            rightMotorGroup = new MotorControllerGroup(rightMotorLeader, rightMotorFollower);

            if (RobotConstants.get().driveMotorType() == MotorType.SPARK_MAX_BRUSHLESS) {
                ((SparkMaxController) leftMotorFollower).setIdleMode(RobotConstants.get().driveIdleMode());
                ((SparkMaxController) rightMotorFollower).setIdleMode(RobotConstants.get().driveIdleMode());
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

    public void arcadeDrive(double speed, double rotation, boolean squareInputs) {
        if (RobotConstants.get().driveUseVelocity()) {
            DifferentialDrive.WheelSpeeds speeds = DifferentialDrive.arcadeDriveIK(MathUtil.applyDeadband(speed, 0.02), MathUtil.applyDeadband(rotation, 0.02), squareInputs);
            setVelocityFromWheelSpeeds(speeds);
        } else {
            diffDrive.arcadeDrive(speed, rotation, squareInputs);
        }
    }

    public void arcadeDrive(double speed, double rotation) {
        arcadeDrive(speed, rotation, true);
    }

    public void curvatureDrive(double speed, double rotation, boolean turnInPlace) {
        if (RobotConstants.get().driveUseVelocity()) {
            DifferentialDrive.WheelSpeeds speeds = DifferentialDrive.curvatureDriveIK(MathUtil.applyDeadband(speed, 0.02), MathUtil.applyDeadband(rotation, 0.02), turnInPlace);
            setVelocityFromWheelSpeeds(speeds);
        } else {
            diffDrive.curvatureDrive(speed, rotation, turnInPlace);
        }
    }

    private void setVelocityFromWheelSpeeds(DifferentialDrive.WheelSpeeds speeds) {
        double leftVelocity =  speeds.left * RobotConstants.get().driveMaxVelocity();
        double rightVelocity =  speeds.right * RobotConstants.get().driveMaxVelocity();
        double leftVoltage = driveFF.calculate(leftVelocity);
        double rightVoltage = driveFF.calculate(rightVelocity);

        if (RobotConstants.get().driveUsePID()) {
            leftVoltage += leftDrivePID.calculate(getLeftVelocity(), leftVelocity);
            rightVoltage += rightDrivePID.calculate(getRightVelocity(), rightVelocity);
        }

        tankDriveVolts(leftVoltage, rightVoltage);
    }

    public void curvatureDrive(double speed, double rotation) {
        curvatureDrive(speed, rotation, true);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMotorGroup.setVoltage(leftVolts);
        rightMotorGroup.setVoltage(rightVolts);
        diffDrive.feed();
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

    public void stop() {
        diffDrive.tankDrive(0, 0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("Motor Left Position", () -> leftMotorLeader.getPosition(), null);
        builder.addDoubleProperty("Motor Left Velocity", () -> leftMotorLeader.getVelocity(), null);
        builder.addDoubleProperty("Motor Right Position", () -> rightMotorLeader.getPosition(), null);
        builder.addDoubleProperty("Motor Right Velocity", () -> rightMotorLeader.getVelocity(), null);
    }

    @Override
    public void initializeTunerNetworkTables(ShuffleboardTab tab) {
        addEntry("speed", tab.add("Driving Speed", 0).withWidget(BuiltInWidgets.kNumberSlider).withSize(2, 1).withPosition(4, 1).withProperties(Map.of("min", -1, "max", 1)).getEntry());
        addEntry("turn", tab.add("Turning Speed", 0).withWidget(BuiltInWidgets.kNumberSlider).withSize(2, 1).withPosition(4, 2).withProperties(Map.of("min", -1, "max", 1)).getEntry());
        addEntry("run", tab.add("Run", false).withWidget(BuiltInWidgets.kToggleButton).withSize(2, 1).withPosition(4, 3).getEntry());
    }

    @Override
    public void initializeTuner() {
        getEntry("speed").setDouble(0);
        getEntry("turn").setDouble(0);
        getEntry("run").setBoolean(false);

        this.setDefaultCommand(new DrivetrainStopCMD(this));
        new NetworkButton(getEntry("run")).whileActiveContinuous(
        new ArcadeDriveCMD(this, 
            () -> getEntry("speed").getDouble(0), 
            () -> getEntry("turn").getDouble(0)
        ));
    }
}
