package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.commands.ArcadeDriveCMD;
import frc.robot.motors.SparkMaxController;
import frc.robot.motors.MotorControllerEncoder;
import frc.robot.motors.MotorControllerFactory;
import frc.robot.motors.TalonController;
import frc.robot.tuning.SubsystemTuner;
import frc.robot.tuning.TunerButtonFactory;
import frc.robot.tuning.TunerParameter;
import frc.robot.tuning.TunerParamterFactory;

public class Drivetrain extends SubsystemTuner {
    MotorControllerGroup leftMotorGroup;
    MotorControllerEncoder leftMotorLeader;
    MotorControllerEncoder leftMotorFollower = null;

    MotorControllerGroup rightMotorGroup;
    MotorControllerEncoder rightMotorLeader;
    MotorControllerEncoder rightMotorFollower = null;
    
    DifferentialDrive diffDrive;

    public Drivetrain() {
        super();

        leftMotorLeader = MotorControllerFactory.create(RobotConstants.get().driveMotorLeftLeaderId(), RobotConstants.get().driveMotorType());
        rightMotorLeader = MotorControllerFactory.create(RobotConstants.get().driveMotorRightLeaderId(), RobotConstants.get().driveMotorType());

        leftMotorLeader.setInverted(RobotConstants.get().driveMotorLeftLeaderInverted());
        // No longer auto inverted by diff drive, you must invert it
        rightMotorLeader.setInverted(RobotConstants.get().driveMotorRightLeaderInverted());

        if (RobotConstants.get().driveDualMotors()) {
            leftMotorFollower = MotorControllerFactory.create(RobotConstants.get().driveMotorLeftFollowerId(), RobotConstants.get().driveMotorType());
            rightMotorFollower = MotorControllerFactory.create(RobotConstants.get().driveMotorRightFollowerId(), RobotConstants.get().driveMotorType());
            
            leftMotorFollower.setInverted(RobotConstants.get().driveMotorLeftFollowerInverted());
            rightMotorFollower.setInverted(RobotConstants.get().driveMotorRightFollowerInverted());

            leftMotorGroup = new MotorControllerGroup(leftMotorLeader, leftMotorFollower);
            rightMotorGroup = new MotorControllerGroup(rightMotorLeader, rightMotorFollower);
        } else {
            leftMotorGroup = new MotorControllerGroup(leftMotorLeader);
            rightMotorGroup = new MotorControllerGroup(rightMotorLeader);
        }

        diffDrive = new DifferentialDrive(leftMotorGroup, rightMotorGroup);
    }

    public void arcadeDrive(double speed, double rotation) {
        diffDrive.arcadeDrive(speed, rotation);
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
    public TunerParameter[] getTunerParameters() {
        return null;
    }

    @Override
    public void initalizeTunerNetworkTables() {
        addTunerParameter("speed", TunerParamterFactory.create("Driving Speed", this, NetworkTableType.kDouble));
        addTunerParameter("turn", TunerParamterFactory.create("Turning Speed", this, NetworkTableType.kDouble));
        addTunerButton("run", TunerButtonFactory.create("Run", this));
    }

    @Override
    public void initalizeTuner() {
        getTunerButton("run").whileActiveContinuous(
        new ArcadeDriveCMD(this, 
            () -> getTunerParameter("speed").getValue().getDouble(), 
            () -> getTunerParameter("turn").getValue().getDouble()
        ));
    }
}
