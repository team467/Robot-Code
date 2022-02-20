package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotConstants;
import frc.robot.motors.SimpleFeedforwardConstant;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gyro;

public class TurnAngleCMD extends CommandBase {
    private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            // RobotConstants.get().driveMaxVelocity(),
            1,
            // RobotConstants.get().driveMaxAcceleration());
            1);

    private final Drivetrain drivetrain;
    private final Gyro gyro;
    private final double angle;
    private final SimpleMotorFeedforward turnFF;
    private final ProfiledPIDController turnPID;
    private final Timer timer;

    public TurnAngleCMD(Drivetrain drivetrain, Gyro gyro, double angle) {
        this.drivetrain = drivetrain;
        this.gyro = gyro;
        this.angle = angle;
        this.turnFF = RobotConstants.get().driveTurnFF().getFeedforward();
        this.turnPID = RobotConstants.get().driveTurnVelocityPID()
                .getProfiledPIDController(constraints);
        timer = new Timer();
        timer.reset();
        timer.start();

        addRequirements(drivetrain);
        addRequirements(gyro);
    }

    @Override
    public void initialize() {
        drivetrain.resetLeftPosition();
        drivetrain.resetRightPosition();
        turnPID.reset(0);
        turnPID.setGoal(angle);
        timer.reset();
    }

    @Override
    public void execute() {
        drivetrain.arcadeDrive(0, turnPID.calculate(gyro.getRotation2d().getDegrees(), angle));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.tankDriveVolts(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (!turnPID.atGoal()) {
            timer.reset();
        }

        return timer.hasElapsed(0.1);
    }
}
