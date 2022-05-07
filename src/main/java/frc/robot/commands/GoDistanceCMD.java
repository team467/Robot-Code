package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotConstants;
import frc.robot.subsystems.Drivetrain;

public class GoDistanceCMD extends CommandBase {
    private final Drivetrain drivetrain;
    private final double distance;
    private final ProfiledPIDController leftPID;
    private final ProfiledPIDController rightPID;
    private final SimpleMotorFeedforward driveFF;
    private final Timer timer;

    private double lastLeftSpeed = 0;
    private double lastRightSpeed = 0;
    private double lastTime = 0;

    public GoDistanceCMD(Drivetrain drivetrain, double distance) {
        this.drivetrain = drivetrain;
        this.distance = distance;
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
                RobotConstants.get().driveAutoMaxVelocity(),
                RobotConstants.get().driveAutoMaxAcceleration());
        this.leftPID = RobotConstants.get().driveDrivePositionPID()
                .getProfiledPIDController(constraints);
        this.rightPID = RobotConstants.get().driveDrivePositionPID()
                .getProfiledPIDController(constraints);
        this.driveFF = RobotConstants.get().driveDriveFF().getFeedforward();
        timer = new Timer();
        timer.reset();
        timer.start();

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.resetPositions();
        leftPID.reset(0);
        rightPID.reset(0);
        leftPID.setGoal(distance);
        rightPID.setGoal(distance);
        leftPID.setTolerance(0.01, 0.01);
        lastTime = Timer.getFPGATimestamp();
        lastLeftSpeed = drivetrain.getLeftVelocity();
        lastRightSpeed = drivetrain.getRightVelocity();
        timer.reset();
    }

    @Override
    public void execute() {
        System.out.printf("Position: %f, Veclocity: %f%n", leftPID.getSetpoint().position, leftPID.getSetpoint().velocity);
        double leftAcceleration = (leftPID.getSetpoint().velocity - lastLeftSpeed)
                / (Timer.getFPGATimestamp() - lastTime);
        double rightAcceleration = (rightPID.getSetpoint().velocity - lastRightSpeed)
                / (Timer.getFPGATimestamp() - lastTime);

        double leftVoltage = MathUtil.clamp(leftPID.calculate(drivetrain.getLeftPosition(), distance), -1, 1)
                + driveFF.calculate(leftPID.getSetpoint().velocity, leftAcceleration);
        double rightVoltage = MathUtil.clamp(rightPID.calculate(drivetrain.getRightPosition(), distance), -1, 1)
                + driveFF.calculate(rightPID.getSetpoint().velocity, rightAcceleration);

        drivetrain.tankDriveVolts(leftVoltage, rightVoltage);
        lastLeftSpeed = drivetrain.getLeftVelocity();
        lastRightSpeed = drivetrain.getRightVelocity();
        lastTime = Timer.getFPGATimestamp();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.tankDriveVolts(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (!(leftPID.atGoal() || rightPID.atGoal())) {
            timer.reset();
        }

        return timer.hasElapsed(0.1);
    }
}
