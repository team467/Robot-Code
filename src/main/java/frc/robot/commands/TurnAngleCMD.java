package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gyro;

public class TurnAngleCMD extends CommandBase {
private Drivetrain drivetrain;
private Gyro gyro;
private double angle;

public TurnAngleCMD(Drivetrain drivetrain, Gyro gyro, double angle) {
    this.drivetrain = drivetrain;
    this.gyro = gyro;
    this.angle = angle;

    addRequirements(drivetrain);
    addRequirements(gyro);
}

@Override
public void initialize() {
    gyro.reset();
}

@Override
public void execute() {
    double error = angle - gyro.getAngle();
    drivetrain.arcadeDrive(0, -MathUtil.clamp(error, -0.3, 0.3));
}

@Override
public void end(boolean interrupted) {
    drivetrain.arcadeDrive(0, 0);
}

// Returns true when the command should end.
@Override
public boolean isFinished() {
    return Math.abs(angle - gyro.getNextAngle()) < 1;
}
}
