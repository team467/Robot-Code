package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotConstants;
import frc.robot.motors.SimpleFeedforwardConstant;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gyro;

public class TurnAngleCMD extends CommandBase {

    private final Drivetrain drivetrain;
    private final Gyro gyro;
    private final double angle;
    private final PIDController turnPID;

    public TurnAngleCMD(Drivetrain drivetrain, Gyro gyro, double angle) {
        this.drivetrain = drivetrain;
        this.gyro = gyro;
        this.angle = angle;
        this.turnPID= RobotConstants.get().driveTurnPositionPID()
                .getPIDController();

        addRequirements(drivetrain);
        addRequirements(gyro);
    }

    @Override
    public void initialize() {
        // TODO check without reset
        drivetrain.resetPositions();
        turnPID.reset();
        // turnPID.enableContinuousInput(-180, 180);
        turnPID.setTolerance(0.1, 1.2);
        turnPID.setSetpoint(angle + gyro.getAngle());
    }

    @Override
    public void execute() {
        drivetrain.arcadeDrive(0, MathUtil.clamp(turnPID.calculate(gyro.getAngle()), -0.5, 0.5));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.tankDriveVolts(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return turnPID.atSetpoint();
    }
}
